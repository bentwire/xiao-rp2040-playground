// Stolen from ranibowio.
use num::PrimInt;

// Tried to make this generic...
// However it isn't because the math would be wrong for anything but u8...
// This should probably be fixed at some point???
pub fn colorwheel<PT>(sel: u8) -> (PT, PT, PT) 
    where
        PT: PrimInt + core::convert::From<u8>,
{
    let r: PT;
    let g: PT;
    let b: PT;
    match sel {
        0..=84 => {
            r = (255 - sel * 3).into();
            g = (sel * 3).into();
            b = 0.into();
        }
        85..=169 => {
            let sel = sel - 85;
            r = 0.into();
            g = (255 - sel * 3).into();
            b = (sel * 3).into();
        }
        170..=255 => {
            let sel = sel - 170;
            r = (sel * 3).into();
            g = 0.into();
            b = (255 - sel * 3).into();
        }
    }
    (r, g, b)
}