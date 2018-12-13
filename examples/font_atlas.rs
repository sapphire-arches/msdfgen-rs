#[macro_use]
extern crate glium;

use euclid::Rect;
use font_kit::font::Font;
use glium::backend::Facade;
use glium::texture::Texture2d;
use glium::{glutin, Surface};
use lyon_path::math::{Angle, Point, Vector};
use lyon_path::Segment;
use msdfgen::{compute_msdf, recolor_contours, Contour, PathCollector};
use std::collections::HashMap;

const SDF_DIMENSION: u32 = 32;

fn get_font() -> Font {
    use font_kit::family_name::FamilyName;
    use font_kit::properties::{Properties, Style};
    use font_kit::source::SystemSource;
    let source = SystemSource::new();

    source
        .select_best_match(
            &[FamilyName::Monospace],
            Properties::new().style(Style::Normal),
        )
        .unwrap()
        .load()
        .unwrap()
}

/// Get a glyph ID for a character, its contours, and the typographic bounds for that glyph
/// TODO: this should also return font.origin() so we can offset the EM-space
/// computations by it. However, on freetype that always returns 0 so for the
/// moment we'll get away without it
fn get_glyph(font: &Font, chr: char) -> (u32, Vec<Contour>, Rect<f32>) {
    use font_kit::hinting::HintingOptions;
    use lyon_path::builder::FlatPathBuilder;
    let glyph_id = font.glyph_for_char(chr).unwrap();

    let mut builder = PathCollector::new();
    font.outline(glyph_id, HintingOptions::None, &mut builder)
        .unwrap();

    (
        glyph_id,
        builder.build(),
        font.typographic_bounds(glyph_id).unwrap(),
    )
}

/// Rescale contours so they fit in the provided rectangle.
/// Returns the scaled contours along with the transformation used to rescale the contours
fn rescale_contours(
    mut contours: Vec<Contour>,
    initial_bounds: Rect<f32>,
    bounds: lyon_path::math::Rect,
) -> (Vec<Contour>, euclid::Transform2D<f32>) {
    let initial_scale = initial_bounds.size.width.max(initial_bounds.size.height);
    let bounds_scale = bounds.size.width.max(bounds.size.height);
    let transformation =
        euclid::Transform2D::create_translation(-initial_bounds.origin.x, -initial_bounds.origin.y)
            .post_scale(bounds_scale / initial_scale, bounds_scale / initial_scale)
            .post_translate(bounds.origin.to_vector());
    for contour in &mut contours {
        for mut elem in &mut contour.elements {
            elem.segment = match elem.segment {
                Segment::Line(s) => Segment::Line(s.transform(&transformation)),
                Segment::Quadratic(s) => Segment::Quadratic(s.transform(&transformation)),
                Segment::Cubic(s) => Segment::Cubic(s.transform(&transformation)),
                Segment::Arc(s) => Segment::Arc(lyon_geom::Arc {
                    center: transformation.transform_point(&s.center),
                    ..s
                }),
            }
        }
    }
    (contours, transformation)
}

#[derive(Copy, Clone)]
struct Vertex2D {
    position: [f32; 2],
    uv: [f32; 2],
    color: [f32; 3],
}

glium::implement_vertex!(Vertex2D, position, uv, color);

/// All the information required to render a character from a string
#[derive(Clone, Copy, Debug)]
struct RenderChar {
    /// The position of the vertices
    verts: Rect<f32>,
    /// The UV coordinates of the vertices
    uv: Rect<f32>,
}

impl RenderChar {
    fn verts(&self) -> [Vertex2D; 4] {
        macro_rules! vertex {
            ($p: expr, $t: expr) => {{
                let color = [rand::random(), rand::random(), rand::random()];
                let p = $p;
                let t = $t;
                Vertex2D {
                    position: [p.x, p.y],
                    uv: [t.x, t.y],
                    color: color.clone(),
                }
            }};
        }

        [
            vertex!(self.verts.bottom_left(), self.uv.bottom_left()),
            vertex!(self.verts.origin, self.uv.origin),
            vertex!(self.verts.bottom_right(), self.uv.bottom_right()),
            vertex!(self.verts.top_right(), self.uv.top_right()),
        ]
    }
}

/// The information about a glyph that gets cached in the font atlas.
/// Since every letter has a different scaling factor to make maximum use of the MSDF pixels,
/// we need to keep track of the offset and scale from font unit space. This
/// information is required when positioning characters to get the right scale
/// and positioning for the geometry.
#[derive(Clone, Copy, Debug)]
struct GlyphInformation {
    id: u32,
    /// Where it actually is in the atlas texture
    uv: Rect<f32>,
    /// The font-space rectangle covered by the uv rectangle
    font_units: Rect<f32>,
}

struct FontAtlas<'font, 'facade, T: Facade> {
    /// Used when a string requires new glyphs
    font: &'font Font,
    /// Reference to the facade that is when we need to grow the atlas texture
    facade: &'facade T,
    /// The scale of each character
    char_dim: u32,
    /// The current dimensions of the texture
    alloced_size: u32,
    /// The x coordinate at which to place the next character,
    next_x: u32,
    /// The y coordinate at which to place the next character,
    next_y: u32,
    /// The actual backing texture that includes all of the distances.
    /// All the distance values should be roughly in [-1, 1]
    tex: Texture2d,
    /// Texture coordinates of every character we know about
    /// Technically, this should probably use glyph ids as keys
    locations: HashMap<char, GlyphInformation>,
}

impl<'font, 'facade, T: Facade> FontAtlas<'font, 'facade, T> {
    /// Create a new atlas.
    fn build(
        char_dim: u32,
        font: &'font Font,
        facade: &'facade T,
    ) -> Result<Self, glium::texture::TextureCreationError> {
        use glium::texture::{MipmapsOption, UncompressedFloatFormat};
        let alloced_size = char_dim * 16;

        let tex = Texture2d::empty_with_format(
            facade,
            UncompressedFloatFormat::F16F16F16,
            MipmapsOption::NoMipmap,
            alloced_size,
            alloced_size,
        )?;

        println!("Allocated {0:?}x{0:?} texture", alloced_size);

        Ok(Self {
            locations: Default::default(),
            next_x: 0,
            next_y: 0,
            font,
            facade,
            char_dim,
            tex,
            alloced_size,
        })
    }

    /// Get the glyph information for a character, either pulling them from the cache
    /// or generating the MSDF
    fn character_information(&mut self, c: char) -> GlyphInformation {
        if !self.locations.contains_key(&c) {
            const INIT_UV_BORDER: f32 = 0.2;
            const UV_BORDER: f32 = 0.1;
            let (glyph_id, contours, font_unit_rect) = get_glyph(self.font, c);
            let uv_rect = Rect::new(
                Point::new(INIT_UV_BORDER, INIT_UV_BORDER),
                euclid::TypedSize2D::new(1.0 - 2.0 * INIT_UV_BORDER, 1.0 - 2.0 * INIT_UV_BORDER),
            );
            let (contours, transform) = rescale_contours(contours, font_unit_rect, uv_rect);

            // Build the contours and upload thfont_unit to the texture
            let contours = recolor_contours(contours, Angle::degrees(3.0), 1);
            let msdf = compute_msdf(&contours, self.char_dim as usize);
            self.tex.write(
                glium::Rect {
                    left: self.next_x,
                    bottom: self.next_y,
                    width: self.char_dim,
                    height: self.char_dim,
                },
                msdf,
            );

            // Compute the final positions of the font_unit and uv rectangles
            // transform should just be a scale and transform, easily invertable
            let inv_transform = transform.inverse().unwrap();
            let uv_rect = Rect::new(
                Point::new(UV_BORDER, UV_BORDER),
                euclid::TypedSize2D::new(1.0 - 2.0 * UV_BORDER, 1.0 - 2.0 * UV_BORDER),
            );
            let font_unit_rect = inv_transform.transform_rect(&uv_rect);
            let alloc_scale = 1.0 / self.alloced_size as f32;
            let uv_rect = uv_rect.scale(
                self.char_dim as f32 * alloc_scale,
                self.char_dim as f32 * alloc_scale,
            );
            let uv_rect = uv_rect
                .translate(&(Vector::new(self.next_x as f32, self.next_y as f32) * alloc_scale));

            // Make sure to advance to the next character slot
            self.next_x += self.char_dim;
            if self.next_x == self.alloced_size {
                self.next_x = 0;
                self.next_y += self.char_dim;
            }

            let tr = GlyphInformation {
                id: glyph_id,
                uv: uv_rect,
                font_units: font_unit_rect,
            };

            self.locations.insert(c, tr);
        }
        self.locations[&c]
    }

    /// Layout a string.
    /// TODO: hide things with interior mutability so that this doesn't take &mut
    fn layout_string(&mut self, start: Point, size_in_em: f32, s: &str) -> Vec<RenderChar> {
        let metrics = self.font.metrics();
        let mut tr = Vec::new();
        let scale = size_in_em / metrics.units_per_em as f32;
        let mut transform = euclid::Transform2D::create_scale(scale, scale)
            .post_translate(start.to_vector() + Vector::new(0.0, metrics.descent * -scale));
        for c in s.chars() {
            let information = self.character_information(c);
            tr.push(RenderChar {
                verts: transform.transform_rect(&information.font_units),
                uv: information.uv,
            });
            transform = transform.post_translate(
                self.font
                    .advance(information.id)
                    .unwrap_or(Vector::new(0.0, 0.0)) * scale,
            );
        }

        tr
    }
}

fn main() {
    let mut events_loop = glutin::EventsLoop::new();
    let mut window_size = (512, 512);
    let window = glutin::WindowBuilder::new().with_dimensions(window_size.into());
    let context = glutin::ContextBuilder::new();
    let context = context.with_gl_profile(glutin::GlProfile::Core);
    let context = context.with_gl_debug_flag(true);
    let display =
        glium::Display::new(window, context, &events_loop).expect("Error creating GL display");
    let hidpi_factor = display.gl_window().window().get_hidpi_factor() as f32;

    let font = get_font();

    let bg_shader = program!(&display,
        410 => {
            vertex: r#"
#version 410
in vec2 position;
in vec2 uv;
in vec3 color;
out vec3 cross_color;
out vec2 cross_uv;

uniform mat4 transform;

void main() {
    gl_Position = vec4(position, 0.0, 1.0) * transform;
    cross_color = color;
    cross_uv = uv;
}"#,
            fragment: r#"
#version 410

uniform sampler2D tex;

in vec2 cross_uv;
in vec3 cross_color;
out vec4 color;

#define RADIUS 0.05

float band_around(float center, float r, float f) {
    return smoothstep(center - r, center, f) -
           smoothstep(center, center + r, f);
}

float remap(float f) {
    return smoothstep(-RADIUS, RADIUS, f);
}

void main() {
    vec3 x = texture(tex, cross_uv).rgb;
    float v = max(min(x.r, x.g), min(max(x.r, x.g), x.b));
    float c = remap(v);
    color = vec4(cross_color.rgb, c);
}"#,
        },
    )
    .unwrap();

    let mut font_atlas =
        FontAtlas::build(SDF_DIMENSION, &font, &display).expect("Failed to build font atlas");
    let layout = font_atlas.layout_string(
        Point::new(30.0, 30.0),
        16.0,
        // ":{<~The lazy cat jumps over the xenophobic dog, yodeling~>}",
        "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789!@#$%^&*()`~'\";/.,<>?",
    );
    eprintln!("{:?}", layout);
    let mut vertices = Vec::with_capacity(layout.len() * 4);
    let mut indices = Vec::with_capacity(layout.len() * 5);
    for c in &layout {
        let base = vertices.len() as u16;
        vertices.extend_from_slice(&c.verts());
        indices.push(base);
        indices.push(base + 1);
        indices.push(base + 2);
        indices.push(base + 3);
        indices.push(std::u16::MAX);
    }

    let tex_vbo = glium::VertexBuffer::immutable(&display, &vertices).unwrap();

    let index_buffer = glium::index::IndexBuffer::new(
        &display,
        glium::index::PrimitiveType::TriangleStrip,
        &indices,
    )
    .unwrap();

    let mut closed = false;
    while !closed {
        let params = glium::DrawParameters {
            blend: glium::Blend::alpha_blending(),
            primitive_restart_index: true,
            ..Default::default()
        };

        let transform = euclid::Transform3D::create_translation(0.0, 0.0, 0.0)
            .pre_translate(euclid::Vector3D::new(-1.0, -1.0, 0.0))
            .pre_scale(2.0 / (window_size.0 as f32), 2.0 / (window_size.1 as f32), 1.0)
            .pre_scale(1.0 / hidpi_factor, 1.0 / hidpi_factor, 1.0 / hidpi_factor);

        let mut target = display.draw();
        target.clear_color(0.0, 0.0, 0.0, 1.0);

        let uniforms = uniform!(
            tex: font_atlas.tex.sampled(),
            transform: transform.to_column_arrays(),
        );

        target
            .draw(&tex_vbo, &index_buffer, &bg_shader, &uniforms, &params)
            .unwrap();

        target.finish().unwrap();

        events_loop.poll_events(|ev| match ev {
            glutin::Event::WindowEvent { event, .. } => match event {
                glutin::WindowEvent::CloseRequested => closed = true,
                glutin::WindowEvent::KeyboardInput { input, .. } => match input.virtual_keycode {
                    _ => {}
                },
                glutin::WindowEvent::Resized(new_size) => {
                    window_size = (new_size.width as u32, new_size.height as u32)
                }
                _ => {}
            },
            _ => {}
        })
    }
}
