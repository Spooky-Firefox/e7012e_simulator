use anyhow::{Context, Result, bail};
use serde::Deserialize;

use crate::geometry::{Segment, Vec2, point_segment_distance, ray_segment_intersection_distance};

#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum LineKind {
    Generic,
    Vertical,
    Horizontal,
}

#[derive(Clone, Debug)]
pub struct MapLine {
    #[allow(dead_code)]
    pub id: String,
    pub kind: LineKind,
    pub segment: Segment,
}

#[derive(Clone, Debug)]
pub struct LoadedMap {
    pub lines: Vec<MapLine>,
}

#[derive(Debug, Deserialize)]
struct MapToml {
    line: Vec<MapLineDef>,
}

#[derive(Debug, Deserialize)]
struct MapLineDef {
    id: Option<String>,
    kind: Option<LineKind>,
    x1: f32,
    y1: f32,
    x2: f32,
    y2: f32,
}

impl LoadedMap {
    pub fn load_from_file(path: &str) -> Result<Self> {
        let raw = std::fs::read_to_string(path)
            .with_context(|| format!("failed to read map file: {path}"))?;
        let parsed: MapToml = toml::from_str(&raw)
            .with_context(|| format!("failed to parse TOML map file: {path}"))?;

        if parsed.line.is_empty() {
            bail!("map has no lines: {path}");
        }

        let mut lines = Vec::with_capacity(parsed.line.len());
        for (idx, def) in parsed.line.into_iter().enumerate() {
            let segment = Segment {
                a: Vec2::new(def.x1, def.y1),
                b: Vec2::new(def.x2, def.y2),
            };
            if segment.a.sub(segment.b).norm() <= 1e-6 {
                bail!("line {} has zero length", idx);
            }
            lines.push(MapLine {
                id: def.id.unwrap_or_else(|| format!("line_{idx}")),
                kind: def.kind.unwrap_or(LineKind::Generic),
                segment,
            });
        }

        Ok(Self { lines })
    }

    pub fn raycast_distance(&self, origin: Vec2, dir: Vec2, max_range_m: f32) -> Option<f32> {
        self.lines
            .iter()
            .filter_map(|line| ray_segment_intersection_distance(origin, dir, line.segment))
            .filter(|distance| *distance <= max_range_m)
            .min_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal))
    }

    pub fn nearest_axis_line(&self, point: Vec2) -> Option<(&MapLine, f32)> {
        self.lines
            .iter()
            .filter(|line| matches!(line.kind, LineKind::Vertical | LineKind::Horizontal))
            .map(|line| (line, point_segment_distance(point, line.segment)))
            .min_by(|(_, d1), (_, d2)| d1.partial_cmp(d2).unwrap_or(std::cmp::Ordering::Equal))
    }

    pub fn bounds(&self) -> (f32, f32, f32, f32) {
        let mut min_x = f32::INFINITY;
        let mut min_y = f32::INFINITY;
        let mut max_x = f32::NEG_INFINITY;
        let mut max_y = f32::NEG_INFINITY;

        for line in &self.lines {
            min_x = min_x.min(line.segment.a.x).min(line.segment.b.x);
            min_y = min_y.min(line.segment.a.y).min(line.segment.b.y);
            max_x = max_x.max(line.segment.a.x).max(line.segment.b.x);
            max_y = max_y.max(line.segment.a.y).max(line.segment.b.y);
        }

        (min_x, min_y, max_x, max_y)
    }
}

impl LineKind {
    pub fn as_str(self) -> &'static str {
        match self {
            LineKind::Generic => "generic",
            LineKind::Vertical => "vertical",
            LineKind::Horizontal => "horizontal",
        }
    }
}
