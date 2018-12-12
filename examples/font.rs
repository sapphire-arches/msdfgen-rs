#[macro_use]
extern crate glium;

use font_kit::font::Font;
use glium::{glutin, Surface};
use lyon_path::math::{Angle, Point, Vector};
use lyon_path::Segment;
use msdfgen::{compute_msdf, recolor_contours, Contour, PathCollector};

const SDF_DIMENSION: usize = 24;

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

fn get_glyph(font: &Font, chr: char) -> Vec<Contour> {
    use font_kit::hinting::HintingOptions;
    use lyon_path::builder::FlatPathBuilder;
    let glyph_id = font.glyph_for_char(chr).unwrap();

    let mut builder = PathCollector::new();
    font.outline(glyph_id, HintingOptions::None, &mut builder)
        .unwrap();

    builder.build()
}

fn bounds_for_contours(contours: &[Contour]) -> euclid::TypedRect<f32> {
    use euclid::{TypedRect, TypedSize2D};
    let mut elements = contours.iter().flat_map(|x| x.elements.iter());
    let initial = {
        let p = match elements.next() {
            Some(x) => x.sample(0.0),
            None => return TypedRect::new(Point::new(0.0, 0.0), TypedSize2D::new(0.0, 0.0)),
        };
        TypedRect::new(p, TypedSize2D::new(0.0, 0.0))
    };
    elements.fold(initial, |acc, x| match x.segment {
        Segment::Line(s) => acc.union(&s.bounding_rect()),
        Segment::Quadratic(s) => acc.union(&s.bounding_rect()),
        Segment::Cubic(s) => acc.union(&s.bounding_rect()),
        Segment::Arc(s) => acc.union(&s.bounding_rect()),
    })
}

fn rescale_contours(mut contours: Vec<Contour>, bounds: lyon_path::math::Rect) -> Vec<Contour> {
    let initial_bounds = bounds_for_contours(&contours);
    let initial_scale = initial_bounds.size.width.max(initial_bounds.size.height);
    let bounds_scale = bounds.size.width.max(bounds.size.height);
    let transformation =
        euclid::Transform2D::create_translation(-initial_bounds.origin.x, -initial_bounds.origin.y)
            .post_scale(bounds_scale / initial_scale, bounds_scale / initial_scale)
            .post_translate(bounds.origin.to_vector());
    for mut contour in &mut contours {
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
    contours
}

#[derive(Copy, Clone)]
struct Vertex2D {
    position: [f32; 2],
    color: [f32; 3],
}

glium::implement_vertex!(Vertex2D, position, color);

enum ContourColorMode {
    TraceContour,
    EdgeColors,
}

fn contours_vbo(
    contours: &[Contour],
    facade: &impl glium::backend::Facade,
    color_mode: ContourColorMode,
) -> glium::VertexBuffer<Vertex2D> {
    const VERTS_PER_SEG: usize = 4;
    let mut verts = Vec::new();
    verts.reserve(contours.iter().map(|c| c.elements.len()).sum::<usize>() * VERTS_PER_SEG);

    for contour in contours {
        let c_color: [f32; 3] = {
            use rand::Rng;
            let mut rng = rand::thread_rng();
            let r = rng.gen_range(0.0, 1.0);
            let g = rng.gen_range(0.0, 1.0);
            let b = rng.gen_range(0.0, 1.0);
            [r, g, b]
        };
        let n_elems = contour.elements.len();
        for (j, elem) in contour.elements.iter().enumerate() {
            let cf = (j as f32) / (n_elems as f32);
            for i in 0..VERTS_PER_SEG {
                let f = (i as f32) / (VERTS_PER_SEG as f32);
                let p = elem.sample(f);
                let cf = cf + (f / (n_elems as f32));
                verts.push(Vertex2D {
                    position: [p.x, p.y],
                    color: match color_mode {
                        ContourColorMode::TraceContour => {
                            [c_color[0] * cf, c_color[1] * cf, c_color[2] * cf]
                        }
                        ContourColorMode::EdgeColors => elem.color.float_color(),
                    },
                });
            }
        }
    }

    glium::VertexBuffer::immutable(facade, &verts).unwrap()
}

struct RenderData {
    raw_contours_vbo: glium::VertexBuffer<Vertex2D>,
    contours_vbo: glium::VertexBuffer<Vertex2D>,
    dist_tex: glium::texture::texture2d::Texture2d,
}

fn contour_vbos_for_chr(
    font: &Font,
    chr: char,
    display: &impl glium::backend::Facade,
) -> RenderData {
    let metrics = font.metrics();
    let contours = get_glyph(&font, chr);
    let contours = rescale_contours(
        contours,
        euclid::TypedRect::new(Point::new(0.2, 0.2), euclid::TypedSize2D::new(0.6, 0.6)),
    );
    let raw_contours_vbo = contours_vbo(&contours, display, ContourColorMode::TraceContour);
    println!("{:?}", contours);
    let contours = recolor_contours(contours, Angle::degrees(3.0), 1);
    println!("{:?}", font.metrics());
    let msdf = compute_msdf(&contours, SDF_DIMENSION);
    use glium::texture::{MipmapsOption, UncompressedFloatFormat};
    let msdf = glium::texture::texture2d::Texture2d::with_format(
        display,
        msdf,
        UncompressedFloatFormat::F16F16F16,
        MipmapsOption::NoMipmap,
    )
    .unwrap();

    let contours_vbo = contours_vbo(&contours, display, ContourColorMode::EdgeColors);

    RenderData {
        dist_tex: msdf,
        raw_contours_vbo,
        contours_vbo,
    }
}

fn main() {
    let mut events_loop = glutin::EventsLoop::new();
    let window = glutin::WindowBuilder::new().with_dimensions((256, 256).into());
    let context = glutin::ContextBuilder::new();
    let context = context.with_gl_profile(glutin::GlProfile::Core);
    let context = context.with_gl_debug_flag(true);
    let display =
        glium::Display::new(window, context, &events_loop).expect("Error creating GL display");

    let font = get_font();

    let line_shader = program!(&display,
        410 => {
            vertex: r#"
#version 410
in vec2 position;
in vec3 color;
out vec3 cross_color;

void main() {
    gl_Position = vec4((position - vec2(0.5, 0.5)) * 2.0, 0.0, 1.0);
    cross_color = color;
}
"#,
            fragment: r#"
#version 410

in vec3 cross_color;
out vec4 color;

void main() {
    color = vec4(cross_color, 1.0);
}

"#,
        },
    )
    .unwrap();

    let bg_shader = program!(&display,
        410 => {
            vertex: r#"
#version 410
in vec2 position;
in vec3 color;
out vec3 cross_color;
out vec2 cross_uv;

void main() {
    gl_Position = vec4((position - vec2(0.5, 0.5)) * 2.0, 0.0, 1.0);
    cross_color = color;
    cross_uv = position;
}"#,
            fragment: r#"
#version 410

uniform sampler2D tex;

in vec2 cross_uv;
in vec3 cross_color;
out vec4 color;

#define RADIUS 0.001
// #define MULTICOLOR
// #define COLOR_SIDES

float band_around(float center, float r, float f) {
    return smoothstep(center - r, center, f) -
           smoothstep(center, center + r, f);
}

float remap(float f) {
    // return 0.5 * f;

    // return 0.25 * band_around(0.45, RADIUS, f) +
    //        0.25 * band_around(0.55, RADIUS, f) +
    //        0.5 * band_around(0.5, RADIUS, f);

    // return band_around(0.5, RADIUS, f);

    return smoothstep(-RADIUS, RADIUS, f);
}

void main() {
    vec3 x = texture(tex, cross_uv).rgb;
#ifdef MULTICOLOR
    x.r = remap(x.r);
    x.g = remap(x.g);
    x.b = remap(x.b);
    color = vec4(x, 1.0);
#else
#ifdef COLOR_SIDES
    float v = x.r;
    if (v > 0.5) {
        color = vec4(1.0 - (vec2(v) * 2.0 - 1.0), 0.0, 1.0);
    } else {
        color = vec4(0.0, vec2(v) * 2.0, 1.0);
    }
#else
    float v = max(min(x.r, x.g), min(max(x.r, x.g), x.b));
    // float c = smoothstep(0.5 - RADIUS, 0.5 + RADIUS, texture(tex, cross_uv).r);
    float c = remap(v);
    color = vec4(vec3(c), 1.0);
#endif
#endif
}"#,
        },
    )
    .unwrap();

    let tex_vbo = glium::VertexBuffer::immutable(
        &display,
        &[
            Vertex2D {
                position: [0.0, 0.0],
                color: [1.0, 1.0, 1.0],
            },
            Vertex2D {
                position: [1.0, 0.0],
                color: [1.0, 1.0, 1.0],
            },
            Vertex2D {
                position: [0.0, 1.0],
                color: [1.0, 1.0, 1.0],
            },
            Vertex2D {
                position: [1.0, 1.0],
                color: [1.0, 1.0, 1.0],
            },
        ],
    )
    .unwrap();

    const DEFAULT_CHAR: char = '4';
    let mut render_data = contour_vbos_for_chr(&font, DEFAULT_CHAR, &display);

    let mut o_down = false;
    let mut draw_outlines = false;

    let mut closed = false;
    while !closed {
        let params = glium::DrawParameters {
            ..Default::default()
        };

        let mut target = display.draw();
        target.clear_color(0.5, 0.5, 0.5, 1.0);

        let uniforms = uniform!(
            tex: render_data
                .dist_tex.sampled()
                .wrap_function(glium::uniforms::SamplerWrapFunction::Clamp),
        );

        target
            .draw(
                &tex_vbo,
                glium::index::NoIndices(glium::index::PrimitiveType::TriangleStrip),
                &bg_shader,
                &uniforms,
                &params,
            )
            .unwrap();

        // let uniforms = uniform!();
        // target
        //     .draw(
        //         &render_data.raw_contours_vbo,
        //         glium::index::NoIndices(glium::index::PrimitiveType::LineStrip),
        //         &line_shader,
        //         &uniforms,
        //         &params,
        //     )
        //     .unwrap();

        if draw_outlines {
            target
                .draw(
                    &render_data.contours_vbo,
                    glium::index::NoIndices(glium::index::PrimitiveType::LineStrip),
                    &line_shader,
                    &uniforms,
                    &params,
                )
                .unwrap();
        }

        target.finish().unwrap();

        events_loop.poll_events(|ev| match ev {
            glutin::Event::WindowEvent { event, .. } => match event {
                glutin::WindowEvent::CloseRequested => closed = true,
                glutin::WindowEvent::KeyboardInput { input, .. } => {
                    let key = match input.virtual_keycode {
                        Some(glutin::VirtualKeyCode::Key0) => '0',
                        Some(glutin::VirtualKeyCode::Key1) => '1',
                        Some(glutin::VirtualKeyCode::Key2) => '2',
                        Some(glutin::VirtualKeyCode::Key3) => '3',
                        Some(glutin::VirtualKeyCode::Key4) => '4',
                        Some(glutin::VirtualKeyCode::Key5) => '5',
                        Some(glutin::VirtualKeyCode::Key6) => '6',
                        Some(glutin::VirtualKeyCode::Key7) => '7',
                        Some(glutin::VirtualKeyCode::Key8) => '8',
                        Some(glutin::VirtualKeyCode::Key9) => '9',
                        Some(glutin::VirtualKeyCode::A) => 'a',
                        Some(glutin::VirtualKeyCode::B) => 'b',
                        Some(glutin::VirtualKeyCode::C) => 'c',
                        Some(glutin::VirtualKeyCode::D) => 'd',
                        Some(glutin::VirtualKeyCode::W) => 'w',
                        Some(glutin::VirtualKeyCode::X) => 'x',
                        Some(glutin::VirtualKeyCode::Y) => 'y',
                        Some(glutin::VirtualKeyCode::Z) => 'z',
                        Some(glutin::VirtualKeyCode::O) => {
                            if !o_down {
                                draw_outlines = !draw_outlines;
                            }
                            o_down = !o_down;
                            DEFAULT_CHAR
                        }
                        _ => DEFAULT_CHAR,
                    };
                    render_data = contour_vbos_for_chr(&font, key, &display);
                }
                _ => {}
            },
            _ => {}
        })
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use lyon_geom::LineSegment;

    fn line_segment(p0: Point, p1: Point) -> Segment {
        Segment::Line(LineSegment { from: p0, to: p1 })
    }

    fn line_element(p0: Point, p1: Point) -> PathElement {
        PathElement::new(line_segment(p0, p1), ColorFlags::W)
    }

    #[test]
    fn test_line_dist_x() {
        let seg = line_element(Point::new(0.0, 0.0), Point::new(2.0, 0.0));
        let (dst, f) = seg.distance(Point::new(1.0, 1.0));
        assert!((dst.distance + 1.0).abs() < 0.0001);
        assert!((f - 0.5).abs() < 0.0001);

        let (dst, f) = seg.distance(Point::new(1.0, -1.0));
        assert!((dst.distance - 1.0).abs() < 0.0001);
        assert!((f - 0.5).abs() < 0.0001);

        let (dst, f) = seg.distance(Point::new(-1.0, 0.0));
        assert!((dst.distance + 1.0).abs() < 0.0001);
        assert!((f + 0.5).abs() < 0.0001);

        let (dst, f) = seg.distance(Point::new(3.0, 0.0));
        assert!((dst.distance - 1.0).abs() < 0.0001);
        assert!((f - 1.5).abs() < 0.0001);
    }

    #[test]
    fn test_line_dist_y() {
        let seg = line_element(Point::new(0.0, 0.0), Point::new(0.0, 2.0));
        let (dst, f) = seg.distance(Point::new(1.0, 1.0));
        assert!((dst.distance - 1.0).abs() < 0.0001);
        assert!((f - 0.5).abs() < 0.0001);

        let (dst, f) = seg.distance(Point::new(-1.0, 1.0));
        assert!((dst.distance + 1.0).abs() < 0.0001);
        assert!((f - 0.5).abs() < 0.0001);

        let (dst, f) = seg.distance(Point::new(0.0, -1.0));
        assert!((dst.distance - 1.0).abs() < 0.0001);
        assert!((f + 0.5).abs() < 0.0001);

        let (dst, f) = seg.distance(Point::new(0.0, 3.0));
        assert!((dst.distance - 1.0).abs() < 0.0001);
        assert!((f - 1.5).abs() < 0.0001);
    }
}
