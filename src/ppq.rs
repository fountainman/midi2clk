pub enum Ppq {
    Ppq1,
    Ppq2,
    Ppq4,
    Ppq24,
    Ppq48,
}

#[allow(dead_code)]
impl Ppq {
    pub fn to_str(&self) -> &'static str {
        match self {
            Ppq::Ppq1 => "Beat",
            Ppq::Ppq2 => "2ppq",
            Ppq::Ppq4 => "4ppq",
            Ppq::Ppq24 => "24ppq",
            Ppq::Ppq48 => "48ppq",
        }
    }
    pub fn to_u8(&self) -> u8 {
        match self {
            Ppq::Ppq1 => 1,
            Ppq::Ppq2 => 2,
            Ppq::Ppq4 => 4,
            Ppq::Ppq24 => 24,
            Ppq::Ppq48 => 48,
        }
    }
    // This is here to prevent MCU from having to do division in fast sections
    pub fn to_max(&self) -> u8 {
        match self {
            Ppq::Ppq1 => 24,
            Ppq::Ppq2 => 12,
            Ppq::Ppq4 => 6,
            Ppq::Ppq24 => 0,
            Ppq::Ppq48 => 0,
        }
    }
    pub fn next(&self) -> Ppq {
        match self {
            Ppq::Ppq1 => Ppq::Ppq2,
            Ppq::Ppq2 => Ppq::Ppq4,
            Ppq::Ppq4 => Ppq::Ppq24,
            Ppq::Ppq24 => Ppq::Ppq48,
            Ppq::Ppq48 => Ppq::Ppq1,
        }
    }
    pub fn prev(&self) -> Ppq {
        match self {
            Ppq::Ppq1 => Ppq::Ppq48,
            Ppq::Ppq2 => Ppq::Ppq1,
            Ppq::Ppq4 => Ppq::Ppq2,
            Ppq::Ppq24 => Ppq::Ppq4,
            Ppq::Ppq48 => Ppq::Ppq24,
        }
    }
}