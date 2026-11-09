#![no_std]

//! Shared, allocation-free ASCII protocol (Linux and RP2350).
//! Requests: `<seq> <command>\n`, e.g. `42 FWD\n`.
//! Responses: `<seq> <OK|ERR> <STOPPED|FORWARD|BACKWARD>\n`.

use core::fmt::{self, Write};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Command {
    Forward,
    Backward,
    Stop,
    Status,
    Ping,
}

impl Command {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Forward => "FWD",
            Self::Backward => "BACK",
            Self::Stop => "STOP",
            Self::Status => "STATUS",
            Self::Ping => "PING",
        }
    }
    fn parse(s: &str) -> Option<Self> {
        Some(match s {
            "FWD" => Self::Forward,
            "BACK" => Self::Backward,
            "STOP" => Self::Stop,
            "STATUS" => Self::Status,
            "PING" => Self::Ping,
            _ => return None,
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MotionState {
    Stopped,
    Forward,
    Backward,
}

impl MotionState {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Stopped => "STOPPED",
            Self::Forward => "FORWARD",
            Self::Backward => "BACKWARD",
        }
    }
    fn parse(s: &str) -> Option<Self> {
        Some(match s {
            "STOPPED" => Self::Stopped,
            "FORWARD" => Self::Forward,
            "BACKWARD" => Self::Backward,
            _ => return None,
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Request {
    pub seq: u16,
    pub command: Command,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Response {
    pub seq: u16,
    pub ok: bool,
    pub state: MotionState,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ParseError {
    Malformed,
    InvalidSequence,
    InvalidCommand,
    InvalidState,
}

pub fn encode_request(w: &mut impl Write, req: Request) -> fmt::Result {
    writeln!(w, "{} {}", req.seq, req.command.as_str())
}

pub fn encode_response(w: &mut impl Write, resp: Response) -> fmt::Result {
    let status = if resp.ok { "OK" } else { "ERR" };
    writeln!(w, "{} {} {}", resp.seq, status, resp.state.as_str())
}

pub fn decode_request(line: &str) -> Result<Request, ParseError> {
    let mut parts = line.split_ascii_whitespace();
    let seq = parts
        .next()
        .ok_or(ParseError::Malformed)?
        .parse::<u16>()
        .map_err(|_| ParseError::InvalidSequence)?;
    let command = Command::parse(parts.next().ok_or(ParseError::Malformed)?)
        .ok_or(ParseError::InvalidCommand)?;
    if parts.next().is_some() {
        return Err(ParseError::Malformed);
    }
    Ok(Request { seq, command })
}

pub fn decode_response(line: &str) -> Result<Response, ParseError> {
    let mut parts = line.split_ascii_whitespace();
    let seq = parts
        .next()
        .ok_or(ParseError::Malformed)?
        .parse::<u16>()
        .map_err(|_| ParseError::InvalidSequence)?;
    let ok = match parts.next().ok_or(ParseError::Malformed)? {
        "OK" => true,
        "ERR" => false,
        _ => return Err(ParseError::Malformed),
    };
    let state = MotionState::parse(parts.next().ok_or(ParseError::Malformed)?)
        .ok_or(ParseError::InvalidState)?;
    if parts.next().is_some() {
        return Err(ParseError::Malformed);
    }
    Ok(Response { seq, ok, state })
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn request_round_trip() {
        let input = Request {
            seq: 42,
            command: Command::Forward,
        };
        let mut out = std::string::String::new();
        encode_request(&mut out, input).unwrap();
        assert_eq!(out, "42 FWD\n");
        assert_eq!(decode_request(&out), Ok(input));
    }
    #[test]
    fn response_round_trip() {
        let input = Response {
            seq: 65535,
            ok: true,
            state: MotionState::Backward,
        };
        let mut out = std::string::String::new();
        encode_response(&mut out, input).unwrap();
        assert_eq!(decode_response(&out), Ok(input));
    }
    #[test]
    fn rejects_bad_command() {
        assert_eq!(
            decode_request("1 LAUNCH\n"),
            Err(ParseError::InvalidCommand)
        );
    }
}

#[cfg(test)]
extern crate std;
