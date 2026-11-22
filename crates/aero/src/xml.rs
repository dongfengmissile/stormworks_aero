//! Minimal tag scanner for the game's XML (definitions, vehicles).
//!
//! The game writes attribute names a strict parser rejects (`00="1"` in
//! `physics_shape_rotation`), and we only ever need tag names and attributes,
//! never text content, so this walks `<...>` spans and nothing else.

pub struct Tag<'a> {
    pub name: &'a str,
    /// `</name>`
    pub close: bool,
    /// `<name ... />`
    pub empty: bool,
    attrs: &'a str,
}

impl<'a> Tag<'a> {
    pub fn get(&self, key: &str) -> Option<&'a str> {
        let mut rest = self.attrs;
        loop {
            rest = rest.trim_start();
            let eq = rest.find('=')?;
            let name = rest[..eq].trim();
            let after = rest[eq + 1..].trim_start();
            let quote = after.chars().next()?;
            if quote != '"' && quote != '\'' {
                return None;
            }
            let end = after[1..].find(quote)? + 1;
            if name == key {
                return Some(&after[1..end]);
            }
            rest = &after[end + 1..];
        }
    }

    pub fn num<T: std::str::FromStr>(&self, key: &str) -> Option<T> {
        self.get(key)?.trim().parse().ok()
    }

    /// Integer attribute that the game omits when it is zero.
    pub fn int0(&self, key: &str) -> i32 {
        self.get(key)
            .and_then(|v| v.trim().parse::<f64>().ok())
            .map(|v| v as i32)
            .unwrap_or(0)
    }
}

pub fn tags(text: &str) -> impl Iterator<Item = Tag<'_>> {
    let mut rest = text;
    std::iter::from_fn(move || loop {
        let start = rest.find('<')?;
        rest = &rest[start + 1..];
        if rest.starts_with("!--") {
            let end = rest.find("-->")?;
            rest = &rest[end + 3..];
            continue;
        }
        // Attribute values never contain '>' in game files.
        let end = rest.find('>')?;
        let body = &rest[..end];
        rest = &rest[end + 1..];
        if body.starts_with('?') || body.starts_with('!') {
            continue;
        }
        let close = body.starts_with('/');
        let body = body.trim_start_matches('/');
        let empty = body.ends_with('/');
        let body = body.trim_end_matches('/');
        let name_end = body.find(|c: char| c.is_whitespace()).unwrap_or(body.len());
        return Some(Tag { name: &body[..name_end], close, empty, attrs: &body[name_end..] });
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scans_game_style_tags() {
        let s = r#"<?xml version="1.0"?><a x="1"><b 00="1" 01='0'/><!-- <c/> --></a>"#;
        let t: Vec<_> = tags(s).collect();
        assert_eq!(t.len(), 3);
        assert_eq!(t[0].name, "a");
        assert_eq!(t[0].num::<i32>("x"), Some(1));
        assert!(t[1].empty);
        assert_eq!(t[1].get("01"), Some("0"));
        assert_eq!(t[1].int0("02"), 0);
        assert!(t[2].close && t[2].name == "a");
    }
}
