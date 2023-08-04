// Stolen from ranibowio.

pub fn colorwheel(sel: u8) -> (u8, u8, u8) {
    let r: u8;
    let g: u8;
    let b: u8;

    match sel {
        0..=84 => {
            r = 255 - sel * 3;
            g = sel * 3;
            b = 0;
        }
        85..=169 => {
            let sel = sel - 85;
            r = 0;
            g = 255 - sel * 3;
            b = sel * 3;
        }
        170..=255 => {
            let sel = sel - 170;
            r = sel * 3;
            g = 0;
            b = 255 - sel * 3;
        }
    }
    (r, g, b)
}