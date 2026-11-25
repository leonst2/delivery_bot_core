//! Reads x/y/z out of a `sensor_msgs/PointCloud2` without depending on the message type, so it can be tested
//! without ROS. The node copies the message fields into a [`CloudView`].

/// `sensor_msgs/PointField` datatypes this reader accepts for x, y and z.
pub const FLOAT32: u8 = 7;
pub const FLOAT64: u8 = 8;

#[derive(Clone, Debug, PartialEq)]
pub struct Field {
    pub name: String,
    pub offset: usize,
    pub datatype: u8,
}

pub struct CloudView<'a> {
    pub fields: Vec<Field>,
    pub data: &'a [u8],
    pub width: usize,
    pub height: usize,
    pub point_step: usize,
    pub row_step: usize,
    pub is_bigendian: bool,
}

#[derive(Clone, Copy)]
struct Reader {
    offset: usize,
    datatype: u8,
}

impl Reader {
    fn size(&self) -> usize {
        if self.datatype == FLOAT64 {
            8
        } else {
            4
        }
    }

    fn read(&self, point: &[u8], big_endian: bool) -> f64 {
        let bytes = &point[self.offset..self.offset + self.size()];
        match (self.datatype, big_endian) {
            (FLOAT64, false) => f64::from_le_bytes(bytes.try_into().unwrap()),
            (FLOAT64, true) => f64::from_be_bytes(bytes.try_into().unwrap()),
            (_, false) => f32::from_le_bytes(bytes.try_into().unwrap()) as f64,
            (_, true) => f32::from_be_bytes(bytes.try_into().unwrap()) as f64,
        }
    }
}

fn reader(view: &CloudView, name: &str, required: bool) -> Result<Option<Reader>, String> {
    let Some(field) = view.fields.iter().find(|f| f.name == name) else {
        return if required {
            Err(format!("point cloud has no '{name}' field"))
        } else {
            Ok(None)
        };
    };
    if field.datatype != FLOAT32 && field.datatype != FLOAT64 {
        return Err(format!(
            "field '{name}' has datatype {}, expected FLOAT32 or FLOAT64",
            field.datatype
        ));
    }
    let r = Reader {
        offset: field.offset,
        datatype: field.datatype,
    };
    if r.offset + r.size() > view.point_step {
        return Err(format!(
            "field '{name}' does not fit into point_step {}",
            view.point_step
        ));
    }
    Ok(Some(r))
}

/// All finite points as `[x, y, z]`, in storage order. A missing `z` field reads as 0.
pub fn read_xyz(view: &CloudView) -> Result<Vec<[f64; 3]>, String> {
    let x = reader(view, "x", true)?.unwrap();
    let y = reader(view, "y", true)?.unwrap();
    let z = reader(view, "z", false)?;
    if view.width > 0 && view.height > 0 {
        let needed = (view.height - 1) * view.row_step + view.width * view.point_step;
        if view.row_step < view.width * view.point_step || view.data.len() < needed {
            return Err(format!(
                "point cloud data too short: {} bytes for {}x{} points (point_step {}, row_step {})",
                view.data.len(),
                view.width,
                view.height,
                view.point_step,
                view.row_step
            ));
        }
    }
    let mut points = Vec::with_capacity(view.width * view.height);
    for row in 0..view.height {
        for col in 0..view.width {
            let start = row * view.row_step + col * view.point_step;
            let point = &view.data[start..start + view.point_step];
            let p = [
                x.read(point, view.is_bigendian),
                y.read(point, view.is_bigendian),
                z.map_or(0.0, |z| z.read(point, view.is_bigendian)),
            ];
            if p.iter().all(|v| v.is_finite()) {
                points.push(p);
            }
        }
    }
    Ok(points)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn field(name: &str, offset: usize, datatype: u8) -> Field {
        Field {
            name: name.into(),
            offset,
            datatype,
        }
    }

    #[test]
    fn reads_little_endian_xyz_intensity_and_skips_nan() {
        // x, y, z, intensity as float32, point_step 16.
        let points = [
            [1.0f32, 2.0, 0.3, 9.0],
            [f32::NAN, 1.0, 0.0, 0.0],
            [-4.5, 0.25, 0.3, 1.0],
        ];
        let data: Vec<u8> = points
            .iter()
            .flatten()
            .flat_map(|v| v.to_le_bytes())
            .collect();
        let view = CloudView {
            fields: vec![
                field("x", 0, FLOAT32),
                field("y", 4, FLOAT32),
                field("z", 8, FLOAT32),
                field("intensity", 12, FLOAT32),
            ],
            data: &data,
            width: 3,
            height: 1,
            point_step: 16,
            row_step: 48,
            is_bigendian: false,
        };
        let xyz = read_xyz(&view).unwrap();
        assert_eq!(xyz.len(), 2);
        assert_eq!(xyz[0], [1.0, 2.0, 0.30000001192092896]);
        assert_eq!(xyz[1][0], -4.5);
    }

    #[test]
    fn reads_big_endian_float64_in_any_field_order_without_z() {
        let data: Vec<u8> = [7.0f64, 3.0].iter().flat_map(|v| v.to_be_bytes()).collect();
        let view = CloudView {
            fields: vec![field("y", 0, FLOAT64), field("x", 8, FLOAT64)],
            data: &data,
            width: 1,
            height: 1,
            point_step: 16,
            row_step: 16,
            is_bigendian: true,
        };
        assert_eq!(read_xyz(&view).unwrap(), vec![[3.0, 7.0, 0.0]]);
    }

    #[test]
    fn rejects_missing_fields_and_short_data() {
        let view = CloudView {
            fields: vec![field("x", 0, FLOAT32)],
            data: &[0; 8],
            width: 1,
            height: 1,
            point_step: 8,
            row_step: 8,
            is_bigendian: false,
        };
        assert!(read_xyz(&view).is_err());
        let view = CloudView {
            fields: vec![field("x", 0, FLOAT32), field("y", 4, FLOAT32)],
            data: &[0; 8],
            width: 2,
            height: 1,
            point_step: 8,
            row_step: 16,
            is_bigendian: false,
        };
        assert!(read_xyz(&view).is_err());
    }
}
