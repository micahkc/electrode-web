pub const SYNAPSE_LOG_FILE_EXTENSION: &str = "sylg";
pub const SYNAPSE_LOG_FILE_IDENTIFIER: &str = "SYLG";
pub const SYNAPSE_LOG_MIME_TYPE: &str = "application/vnd.synapse.log";

#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct SynapseLogStream {
    records: Vec<Vec<u8>>,
}

impl SynapseLogStream {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn push_size_prefixed_record(&mut self, record: impl Into<Vec<u8>>) {
        self.records.push(record.into());
    }

    pub fn record_count(&self) -> usize {
        self.records.len()
    }

    pub fn is_empty(&self) -> bool {
        self.records.is_empty()
    }

    pub fn into_bytes(self) -> Vec<u8> {
        let total_len = self.records.iter().map(Vec::len).sum();
        let mut bytes = Vec::with_capacity(total_len);
        for record in self.records {
            bytes.extend(record);
        }
        bytes
    }
}
