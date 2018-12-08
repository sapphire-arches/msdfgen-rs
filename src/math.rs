pub(crate) fn median(a: f32, b: f32, c: f32) -> f32 {
    let min = |a: f32, b: f32| a.min(b);
    let max = |a: f32, b: f32| a.max(b);
    max(min(a, b), min(max(a, b), c))
}
