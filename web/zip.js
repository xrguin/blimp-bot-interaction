// Small, uncompressed ZIP writer. PNG frames are already compressed.
const encoder = new TextEncoder();
const crcTable = Uint32Array.from({ length: 256 }, (_, n) => {
  for (let k = 0; k < 8; k += 1) n = (n & 1) ? 0xedb88320 ^ (n >>> 1) : n >>> 1;
  return n >>> 0;
});

export function crc32(bytes) {
  let crc = 0xffffffff;
  for (const byte of bytes) crc = crcTable[(crc ^ byte) & 255] ^ (crc >>> 8);
  return (crc ^ 0xffffffff) >>> 0;
}

export class FrameZip {
  constructor() { this.parts = []; this.entries = []; this.offset = 0; }

  async add(name, data) {
    if (!/^[\w./-]+$/.test(name) || name.includes('..') || name.startsWith('/')) throw new Error('Invalid archive filename');
    const bytes = typeof data === 'string' ? encoder.encode(data)
      : data instanceof Uint8Array ? data : new Uint8Array(await data.arrayBuffer());
    if (this.offset + bytes.length > 1_000_000_000) throw new Error('Recording is too large for this browser export. Use a shorter recording or lower resolution.');
    const filename = encoder.encode(name);
    const header = new Uint8Array(30 + filename.length);
    const view = new DataView(header.buffer);
    const crc = crc32(bytes);
    view.setUint32(0, 0x04034b50, true);
    view.setUint16(4, 20, true);
    view.setUint16(12, 33, true); // ZIP epoch: 1980-01-01.
    view.setUint32(14, crc, true);
    view.setUint32(18, bytes.length, true);
    view.setUint32(22, bytes.length, true);
    view.setUint16(26, filename.length, true);
    header.set(filename, 30);
    this.parts.push(header, bytes);
    this.entries.push({ filename, crc, size: bytes.length, offset: this.offset });
    this.offset += header.length + bytes.length;
  }

  finish() {
    const central = [];
    let centralSize = 0;
    for (const entry of this.entries) {
      const header = new Uint8Array(46 + entry.filename.length);
      const view = new DataView(header.buffer);
      view.setUint32(0, 0x02014b50, true);
      view.setUint16(4, 20, true);
      view.setUint16(6, 20, true);
      view.setUint16(14, 33, true);
      view.setUint32(16, entry.crc, true);
      view.setUint32(20, entry.size, true);
      view.setUint32(24, entry.size, true);
      view.setUint16(28, entry.filename.length, true);
      view.setUint32(42, entry.offset, true);
      header.set(entry.filename, 46);
      central.push(header);
      centralSize += header.length;
    }
    const end = new Uint8Array(22);
    const view = new DataView(end.buffer);
    view.setUint32(0, 0x06054b50, true);
    view.setUint16(8, this.entries.length, true);
    view.setUint16(10, this.entries.length, true);
    view.setUint32(12, centralSize, true);
    view.setUint32(16, this.offset, true);
    return new Blob([...this.parts, ...central, end], { type: 'application/zip' });
  }
}
