use ez80::*;
use std::sync::mpsc::{Sender, Receiver};
use std::sync::mpsc;
use std::collections::HashMap;
use std::io::{ Seek, SeekFrom, Read, Write };
use std::str;

const ROM_SIZE: usize = 0x40000; // 256 KiB
const RAM_SIZE: usize = 0x80000; // 512 KiB
const MEM_SIZE: usize = ROM_SIZE + RAM_SIZE;
const UPPER_MEM_SIZE: usize = 8*1024;

mod mos {
    // FatFS struct FIL
    pub const SIZEOF_MOS_FIL_STRUCT: u32 = 36;
    pub const FIL_MEMBER_OBJSIZE: u32 = 11;
    pub const FIL_MEMBER_FPTR: u32 = 17;
    // FatFS struct FILINFO
    pub const SIZEOF_MOS_FILINFO_STRUCT: u32 = 278;
	pub const FILINFO_MEMBER_FSIZE_U32: u32 = 0;
    //pub const FILINFO_MEMBER_FDATE_U16: u32 = 4;
    //pub const FILINFO_MEMBER_FTIME_U16: u32 = 6;
    pub const FILINFO_MEMBER_FATTRIB_U8: u32 = 8;
    //pub const FILINFO_MEMBER_ALTNAME_13BYTES: u32 = 9;
    pub const FILINFO_MEMBER_FNAME_256BYTES: u32 = 22;
    // f_open mode (3rd arg)
    //pub const FA_READ: u32 = 1;
    pub const FA_WRITE: u32 = 2;
    pub const FA_CREATE_NEW: u32 = 4;
    pub const FA_CREATE_ALWAYS: u32 = 8;
}

struct MosMap {
    pub f_chdir: u32,
    pub _f_chdrive: u32,
    pub f_close: u32,
    pub f_closedir: u32,
    pub _f_getcwd: u32,
    pub _f_getfree: u32,
    pub f_getlabel: u32,
    pub f_gets: u32,
    pub f_lseek: u32,
    pub f_mkdir: u32,
    pub f_mount: u32,
    pub f_open: u32,
    pub f_opendir: u32,
    pub _f_printf: u32,
    pub f_putc: u32,
    pub _f_puts: u32,
    pub f_read: u32,
    pub f_readdir: u32,
    pub f_rename: u32,
    pub _f_setlabel: u32,
    pub f_stat: u32,
    pub _f_sync: u32,
    pub _f_truncate: u32,
    pub f_unlink: u32,
    pub f_write: u32,
}
const DELTA:u32 = 0x4FA1-0x4ebe;
static MOS_103_MAP: MosMap = MosMap {
    f_chdir    : 0x5E79+DELTA,
    _f_chdrive : 0x0000,
    f_close    : 0x5DF2+DELTA,
    f_closedir : 0x6722+DELTA,
    _f_getcwd  : 0x0000,
    _f_getfree : 0x0000,
    f_getlabel : 0x73DD+DELTA,
    f_gets     : 0x7858+DELTA,
    f_lseek    : 0x61D7+DELTA,
    f_mkdir    : 0x6EBD+DELTA,
    f_mount    : 0x4EBE+DELTA,
    f_open     : 0x4F53+DELTA,
    f_opendir  : 0x6619+DELTA,
    _f_printf  : 0x0000,
    f_putc     : 0x0000,
    _f_puts    : 0x0000,
    f_read     : 0x5425+DELTA,
    f_readdir  : 0x6759+DELTA,
    f_rename   : 0x0000,
    _f_setlabel: 0x0000,
    f_stat     : 0x0000,
    _f_sync    : 0x0000,
    _f_truncate: 0x0000,
    f_unlink   : 0x6CE1+DELTA,
    f_write    : 0x57D7+DELTA,
};

pub struct AgonMachine {
    mem: [u8; MEM_SIZE],
    upper_mem: [u8; UPPER_MEM_SIZE],
    tx: Sender<u8>,
    rx: Receiver<u8>,
    rx_buf: Option<u8>,
    // map from MOS fatfs FIL struct ptr to rust File handle
    open_files: HashMap<u32, std::fs::File>,
    open_dirs: HashMap<u32, std::fs::ReadDir>,
    enable_hostfs: bool,
    hostfs_root_dir: std::path::PathBuf,
    mos_current_dir: MosPath,
    vsync_counter: std::sync::Arc<std::sync::atomic::AtomicU32>,
    transmit_interrupt: bool,
    upper_memory_base: u8,
    upper_memory_enabled: bool,
    msxdos_started:bool
}

// a path relative to the hostfs_root_dir
pub struct MosPath(std::path::PathBuf);

impl Machine for AgonMachine {
    fn peek(&self, address: u32) -> u8 {
        if address < 0x20000 || (address >= 0x40000 && address < 0xc0000) {
            if self.upper_memory_enabled && (address>=0x5e000 && address<0x60000) || (address>=0x6e000 && address<0x70000) {
                self.upper_mem[((address & 0xffff)-0xe000) as usize]
            } else {
                self.mem[address as usize]
            }
        } else {
            eprintln!("eZ80 memory read out of bounds: ${:x}", address);
            0xf5
        }
    }

    fn poke(&mut self, address: u32, value: u8) {
        if address < 0x20000 || (address >= 0x40000 && address < 0xc0000) {
            if self.upper_memory_enabled && (address>=0x5e000 && address<0x60000) || (address>=0x6e000 && address<0x70000) {
                self.upper_mem[((address & 0xffff)-0xe000) as usize] = value;
            } else {
                self.mem[address as usize] = value;
            }
        } else {
            eprintln!("eZ80 memory write out of bounds: ${:x}", address);
        }
    }

    fn port_in(&mut self, address: u16) -> u8 {
        //println!("IN({:02X})", address);
        if address == 0xa2 {
            0x0 // UART0 clear to send
        } else if address == 0xc0 {
            // uart0 receive
            self.maybe_fill_rx_buf();

            let maybe_data = self.rx_buf;
            self.rx_buf = None;

            match maybe_data {
                Some(data) => data,
                None => 0
            }
        } else if address == 0xc2 {
            // MSM: IIR
            if self.transmit_interrupt == true {    
                self.transmit_interrupt=false;
                0x02   
            }
            else {    
                0x04    
            }
        } else if address == 0xc5 {
            self.maybe_fill_rx_buf();

            match self.rx_buf {
                Some(_) => 0x41,
                None => 0x40
            }
            // UART_LSR_ETX		EQU 	%40 ; Transmit empty (can send)
            // UART_LSR_RDY		EQU	%01		; Data ready (can receive)
        } else if address == 0xc6 {
            // MSM: CTS
            0x10 
        } else if address == 0xc1 {
            // MSX UART0_IER
            0x00
        } else if address == 0x81 /* timer0 low byte */ {
            std::thread::sleep(std::time::Duration::from_millis(10));
            0x0
        } else if address == 0x82 /* timer0 high byte */ {
            0x0
        } else if address == 0x9a {
            0x0
        } else {
            println!("Unhandled IN x,({:02X}h)",address&0xff);
            0
        }
    }
    fn port_out(&mut self, address: u16, value: u8) {
        //println!("OUT(${:02X}) = ${:x}", address, value);
        if address == 0xc0 /* UART0_REG_THR */ {
            /* Send data to VDP */
            self.tx.send(value).unwrap();

            //print!("{}", char::from_u32(value as u32).unwrap());
            //std::io::stdout().flush().unwrap();
        } else if address == 0xc1 {
            // MSM: IER
            if value & 0x02 != 0 {
                // request transmit interrupt
                self.transmit_interrupt = true;
            }
        } else if address == 0xb4 {
            // RAM_CTL
            if value & 0b10000000 != 0 {
                self.upper_memory_enabled = true;
            }
            else {
                self.upper_memory_enabled = false;
            }
        } else if address == 0xb5 {
            // RAM_ADDR_U
            self.upper_memory_base = value;
        } else if address == 0x9a { 
        } else {
            println!("Unhandled OUT ({:02X}h),{:02X}h",address&0xff,value);
        }
    }
}

impl AgonMachine {
    pub fn new(tx : Sender<u8>, rx : Receiver<u8>, vsync_counter: std::sync::Arc<std::sync::atomic::AtomicU32>) -> AgonMachine {
        AgonMachine {
            mem: [0; MEM_SIZE],
            upper_mem: [0; UPPER_MEM_SIZE],
            tx,
            rx,
            rx_buf: None,
            open_files: HashMap::new(),
            open_dirs: HashMap::new(),
            enable_hostfs: true,
            hostfs_root_dir: std::env::current_dir().unwrap(),
            mos_current_dir: MosPath(std::path::PathBuf::new()),
            vsync_counter,
            transmit_interrupt: false,
            upper_memory_base: 0,
            upper_memory_enabled: false,
            msxdos_started: false,
        }
    }

    pub fn set_sdcard_directory(&mut self, path: std::path::PathBuf) {
        self.hostfs_root_dir = path;
    }

    fn maybe_fill_rx_buf(&mut self) -> Option<u8> {
        if self.rx_buf == None {
            self.rx_buf = match self.rx.try_recv() {
                Ok(data) => Some(data),
                Err(mpsc::TryRecvError::Disconnected) => panic!(),
                Err(mpsc::TryRecvError::Empty) => None
            }
        }
        self.rx_buf
    }

    fn load_mos(&mut self) {
        let code = match std::fs::read("AgonElectronOS.bin") {
            Ok(data) => data,
            Err(e) => {
                eprintln!("Error opening AgonElectronOS.bin: {:?}", e);
                std::process::exit(-1);
            }
        };
        
        for (i, e) in code.iter().enumerate() {
            self.mem[i] = *e;
        }

        // checksum the loaded MOS, to identify supported versions
        //let checksum = z80_mem_tools::checksum(self, 0, code.len() as u32);
        //if checksum != 0xc102d8 {
        //    eprintln!("WARNING: Unsupported MOS version (only 1.03 is supported): disabling hostfs");
        //    self.enable_hostfs = false;
        //}
    }

    fn hostfs_mos_f_getlabel(&mut self, cpu: &mut Cpu) {
        let mut buf = self._peek24(cpu.state.sp() + 6);
        let label = "hostfs";
        for b in label.bytes() {
            self.poke(buf, b);
            buf += 1;
        }
        self.poke(buf, 0);

        // success
        cpu.state.reg.set24(Reg16::HL, 0); // success

        let mut env = Environment::new(&mut cpu.state, self);
        env.subroutine_return();
    }

    fn hostfs_mos_f_close(&mut self, cpu: &mut Cpu) {
        let fptr = self._peek24(cpu.state.sp() + 3);
        //eprintln!("f_close(${:x})", fptr);

        // closes on Drop
        self.open_files.remove(&fptr);

        // success
        cpu.state.reg.set24(Reg16::HL, 0);

        Environment::new(&mut cpu.state, self).subroutine_return();
    }

    fn hostfs_mos_f_gets(&mut self, cpu: &mut Cpu) {
        let buffer = self._peek24(cpu.state.sp() + 3);
        let max_len = self._peek24(cpu.state.sp() + 6);
        let fptr = self._peek24(cpu.state.sp() + 9);
        let mut buf = buffer;

        match self.open_files.get(&fptr) {
            Some(mut f) => {
                let mut line = vec![];
                let mut host_buf = vec![0; 1];
                for _ in 0..max_len {
                    f.read(host_buf.as_mut_slice()).unwrap();
                    line.push(host_buf[0]);

                    if host_buf[0] == 10 { break; } // line end
                    if host_buf[0] == 0 { // end of file
                        cpu.state.reg.set24(Reg16::HL, 0); // error
                        let mut env = Environment::new(&mut cpu.state, self);
                        env.subroutine_return();
                        return;
                    }
                }
                // no f.tell()...
                //let fpos = f.seek(SeekFrom::Current(0)).unwrap();
                // save file position to FIL.fptr U32
                //self._poke24(fptr + mos::FIL_MEMBER_FPTR, fpos as u32);
                for b in line {
                    self.poke(buf, b);
                    buf += 1;
                }
                self.poke(buf, 0);
                cpu.state.reg.set24(Reg16::HL, buffer); // success
            }
            None => {
                cpu.state.reg.set24(Reg16::HL, 0); // error
            }
        }
        let mut env = Environment::new(&mut cpu.state, self);
        env.subroutine_return();
    }

    fn hostfs_mos_f_putc(&mut self, cpu: &mut Cpu) {
        let ch = self._peek24(cpu.state.sp() + 3);
        let fptr = self._peek24(cpu.state.sp() + 6);

        match self.open_files.get(&fptr) {
            Some(mut f) => {
                f.write(&[ch as u8]).unwrap();

                // no f.tell()...
                let fpos = f.seek(SeekFrom::Current(0)).unwrap();
                // save file position to FIL.fptr
                self._poke24(fptr + mos::FIL_MEMBER_FPTR, fpos as u32);

                // success
                cpu.state.reg.set24(Reg16::HL, 0);
            }
            None => {
                // error
                cpu.state.reg.set24(Reg16::HL, 1);
            }
        }

        let mut env = Environment::new(&mut cpu.state, self);
        env.subroutine_return();
    }

    fn hostfs_mos_f_write(&mut self, cpu: &mut Cpu) {
        let fptr = self._peek24(cpu.state.sp() + 3);
        let buf = self._peek24(cpu.state.sp() + 6);
        let num = self._peek24(cpu.state.sp() + 9);
        let num_written_ptr = self._peek24(cpu.state.sp() + 12);
        //eprintln!("f_write(${:x}, ${:x}, {}, ${:x})", fptr, buf, num, num_written_ptr);

        match self.open_files.get(&fptr) {
            Some(mut f) => {
                for i in 0..num {
                    let byte = self.peek(buf + i);
                    f.write(&[byte]).unwrap();
                }

                // no f.tell()...
                let fpos = f.seek(SeekFrom::Current(0)).unwrap();
                // save file position to FIL.fptr
                self._poke24(fptr + mos::FIL_MEMBER_FPTR, fpos as u32);

                // inform caller that all bytes were written
                self._poke24(num_written_ptr, num);

                // success
                cpu.state.reg.set24(Reg16::HL, 0);
            }
            None => {
                // error
                cpu.state.reg.set24(Reg16::HL, 1);
            }
        }

        let mut env = Environment::new(&mut cpu.state, self);
        env.subroutine_return();
    }

    fn hostfs_mos_f_read(&mut self, cpu: &mut Cpu) {
        let fptr = self._peek24(cpu.state.sp() + 3);
        let mut buf = self._peek24(cpu.state.sp() + 6);
        let len = self._peek24(cpu.state.sp() + 9);
        let bytes_read_ptr = self._peek24(cpu.state.sp() + 12);
        //eprintln!("f_read(${:x}, ${:x}, ${:x}, ${:x})", fptr, buf, len, bytes_read_ptr);
        match self.open_files.get(&fptr) {
            Some(mut f) => {
                let mut host_buf: Vec<u8> = vec![0; len as usize];
                let num_bytes_read = f.read(host_buf.as_mut_slice()).unwrap();
                // no f.tell()...
                let fpos = f.seek(SeekFrom::Current(0)).unwrap();
                // copy to agon ram 
                for b in host_buf {
                    self.poke(buf, b);
                    buf += 1;
                }
                // save file position to FIL.fptr
                self._poke24(fptr + mos::FIL_MEMBER_FPTR, fpos as u32);
                // save num bytes read
                self._poke24(bytes_read_ptr, num_bytes_read as u32);

                cpu.state.reg.set24(Reg16::HL, 0); // ok
            }
            None => {
                cpu.state.reg.set24(Reg16::HL, 1); // error
            }
        }
        let mut env = Environment::new(&mut cpu.state, self);
        env.subroutine_return();
    }

    fn hostfs_mos_f_closedir(&mut self, cpu: &mut Cpu) {
        let dir_ptr = self._peek24(cpu.state.sp() + 3);
        // closes on Drop
        self.open_dirs.remove(&dir_ptr);

        // success
        cpu.state.reg.set24(Reg16::HL, 0); // success

        let mut env = Environment::new(&mut cpu.state, self);
        env.subroutine_return();
    }

    fn hostfs_set_filinfo_from_metadata(&mut self, z80_filinfo_ptr: u32, path: &std::path::PathBuf, metadata: &std::fs::Metadata) {
        // XXX to_str can fail if not utf-8
        // write file name
        z80_mem_tools::memcpy_to_z80(
            self, z80_filinfo_ptr + mos::FILINFO_MEMBER_FNAME_256BYTES,
            path.file_name().unwrap().to_str().unwrap().as_bytes()
        );

        // write file length (U32)
        self._poke24(z80_filinfo_ptr + mos::FILINFO_MEMBER_FSIZE_U32, metadata.len() as u32);
        self.poke(z80_filinfo_ptr + mos::FILINFO_MEMBER_FSIZE_U32 + 3, (metadata.len() >> 24) as u8);

        // is directory?
        if metadata.is_dir() {
            self.poke(z80_filinfo_ptr + mos::FILINFO_MEMBER_FATTRIB_U8, 0x10 /* AM_DIR */);
        }

        // TODO set fdate, ftime
    }

    fn hostfs_mos_f_readdir(&mut self, cpu: &mut Cpu) {
        let dir_ptr = self._peek24(cpu.state.sp() + 3);
        let file_info_ptr = self._peek24(cpu.state.sp() + 6);

        // clear the FILINFO struct
        z80_mem_tools::memset(self, file_info_ptr, 0, mos::SIZEOF_MOS_FILINFO_STRUCT);

        match self.open_dirs.get_mut(&dir_ptr) {
            Some(dir) => {

                match dir.next() {
                    Some(Ok(dir_entry)) => {
                        let path = dir_entry.path();
                        if let Ok(metadata) = std::fs::metadata(&path) {
                            self.hostfs_set_filinfo_from_metadata(file_info_ptr, &path, &metadata);

                            // success
                            cpu.state.reg.set24(Reg16::HL, 0);
                        } else {
                            // hm. why might std::fs::metadata fail?
                            z80_mem_tools::memcpy_to_z80(
                                self, file_info_ptr + mos::FILINFO_MEMBER_FNAME_256BYTES,
                                "<error reading file metadata>".as_bytes()
                            );
                            cpu.state.reg.set24(Reg16::HL, 0);
                        }
                    }
                    Some(Err(_)) => {
                        cpu.state.reg.set24(Reg16::HL, 1); // error
                    }
                    None => {
                        // directory has been read to the end.
                        // do nothing, since FILINFO.fname[0] == 0 indicates to MOS that
                        // the directory end has been reached

                        // success
                        cpu.state.reg.set24(Reg16::HL, 0);
                    }
                }
            }
            None => {
                cpu.state.reg.set24(Reg16::HL, 1); // error
            }
        }

        Environment::new(&mut cpu.state, self).subroutine_return();
    }

    fn hostfs_mos_f_mkdir(&mut self, cpu: &mut Cpu) {
        let dir_name = unsafe {
            String::from_utf8_unchecked(z80_mem_tools::get_cstring(self, self._peek24(cpu.state.sp() + 3)))
        };
        //eprintln!("f_mkdir(\"{}\")", dir_name);

        match std::fs::create_dir(self.host_path_from_mos_path_join(&dir_name)) {
            Ok(_) => {
                // success
                cpu.state.reg.set24(Reg16::HL, 0);
            }
            Err(_) => {
                // error
                cpu.state.reg.set24(Reg16::HL, 1);
            }
        }
        Environment::new(&mut cpu.state, self).subroutine_return();
    }

    fn hostfs_mos_f_rename(&mut self, cpu: &mut Cpu) {
        let old_name = unsafe {
            String::from_utf8_unchecked(z80_mem_tools::get_cstring(self, self._peek24(cpu.state.sp() + 3)))
        };
        let new_name = unsafe {
            String::from_utf8_unchecked(z80_mem_tools::get_cstring(self, self._peek24(cpu.state.sp() + 6)))
        };
        //eprintln!("f_rename(\"{}\", \"{}\")", old_name, new_name);

        match std::fs::rename(self.host_path_from_mos_path_join(&old_name),
                              self.host_path_from_mos_path_join(&new_name)) {
            Ok(_) => {
                // success
                cpu.state.reg.set24(Reg16::HL, 0);
            }
            Err(e) => {
                match e.kind() {
                    std::io::ErrorKind::NotFound => {
                        cpu.state.reg.set24(Reg16::HL, 4);
                    }
                    _ => {
                        cpu.state.reg.set24(Reg16::HL, 1);
                    }
                }
            }
        }
        Environment::new(&mut cpu.state, self).subroutine_return();
    }

    fn hostfs_mos_f_chdir(&mut self, cpu: &mut Cpu) {
        let cd_to_ptr = self._peek24(cpu.state.sp() + 3);
        let cd_to = unsafe {
            // MOS filenames may not be valid utf-8
            String::from_utf8_unchecked(z80_mem_tools::get_cstring(self, cd_to_ptr))
        };
        //eprintln!("f_chdir({})", cd_to);

        let new_path = self.mos_path_join(&cd_to);

        match std::fs::metadata(self.mos_path_to_host_path(&new_path)) {
            Ok(metadata) => {
                if metadata.is_dir() {
                    //eprintln!("setting path to {:?}", &new_path);
                    self.mos_current_dir = new_path;
                    cpu.state.reg.set24(Reg16::HL, 0);
                } else {
                    cpu.state.reg.set24(Reg16::HL, 1);
                }
            }
            Err(e) => {
                match e.kind() {
                    std::io::ErrorKind::NotFound => {
                        cpu.state.reg.set24(Reg16::HL, 4);
                    }
                    _ => {
                        cpu.state.reg.set24(Reg16::HL, 1);
                    }
                }
            }
        }
        Environment::new(&mut cpu.state, self).subroutine_return();
    }

    fn hostfs_mos_f_mount(&mut self, cpu: &mut Cpu) {
        // always success. hostfs is mounted
        cpu.state.reg.set24(Reg16::HL, 0); // ok
        Environment::new(&mut cpu.state, self).subroutine_return();
    }

    fn hostfs_mos_f_unlink(&mut self, cpu: &mut Cpu) {
        let filename_ptr = self._peek24(cpu.state.sp() + 3);
        let filename = unsafe {
            String::from_utf8_unchecked(z80_mem_tools::get_cstring(self, filename_ptr))
        };
        //eprintln!("f_unlink(\"{}\")", filename);

        match std::fs::remove_file(self.host_path_from_mos_path_join(&filename)) {
            Ok(()) => {
                cpu.state.reg.set24(Reg16::HL, 0); // ok
            }
            Err(e) => {
                match e.kind() {
                    std::io::ErrorKind::NotFound => {
                        cpu.state.reg.set24(Reg16::HL, 4);
                    }
                    _ => {
                        cpu.state.reg.set24(Reg16::HL, 1);
                    }
                }
            }
        };

        Environment::new(&mut cpu.state, self).subroutine_return();
    }

    fn hostfs_mos_f_opendir(&mut self, cpu: &mut Cpu) {
        //fr = f_opendir(&dir, path);
        let dir_ptr = self._peek24(cpu.state.sp() + 3);
        let path_ptr = self._peek24(cpu.state.sp() + 6);
        let path = unsafe {
            // MOS filenames may not be valid utf-8
            String::from_utf8_unchecked(z80_mem_tools::get_cstring(self, path_ptr))
        };
        //eprintln!("f_opendir(${:x}, \"{}\")", dir_ptr, path.trim_end());

        match std::fs::read_dir(self.host_path_from_mos_path_join(&path)) {
            Ok(dir) => {
                // XXX should clear the DIR struct in z80 ram
                
                // store in map of z80 DIR ptr to rust ReadDir
                self.open_dirs.insert(dir_ptr, dir);
                cpu.state.reg.set24(Reg16::HL, 0); // ok
            }
            Err(e) => {
                match e.kind() {
                    std::io::ErrorKind::NotFound => {
                        cpu.state.reg.set24(Reg16::HL, 4);
                    }
                    _ => {
                        cpu.state.reg.set24(Reg16::HL, 1);
                    }
                }
            }
        }

        cpu.state.reg.set24(Reg16::HL, 0); // ok
        let mut env = Environment::new(&mut cpu.state, self);
        env.subroutine_return();
    }

    fn mos_path_to_host_path(&mut self, path: &MosPath) -> std::path::PathBuf {
        self.hostfs_root_dir.join(&path.0)
    }

    /**
     * Return a new MosPath, `new_fragments` joined to mos_current_dir
     */
    fn mos_path_join(&mut self, new_fragments: &str) -> MosPath {
        let mut full_path = self.mos_current_dir.0.clone();
        let new_fragments_path = std::path::PathBuf::from(new_fragments.trim_end());

        for fragment in &new_fragments_path {
            match fragment.to_str().unwrap() {
                "." => {}
                ".." => {
                    full_path.pop();
                }
                "/" => {
                    full_path = std::path::PathBuf::new();
                }
                f => {
                    let abs_path = self.hostfs_root_dir.join(&full_path);
                    
                    // look for a case-insensitive match for this path fragment
                    let matched_f = match std::fs::read_dir(abs_path) {
                        Ok(dir) => {
                            if let Some(ci_f) = dir.into_iter().find(|item| {
                                match item {
                                    Ok(dir_entry) => dir_entry.file_name().to_ascii_lowercase().into_string() == Ok(f.to_ascii_lowercase()),
                                    Err(_) => false
                                }
                            }) {
                                // found a case-insensitive match
                                ci_f.unwrap().file_name()
                            } else {
                                std::ffi::OsString::from(f)
                            }
                        }
                        Err(_) => {
                            std::ffi::OsString::from(f)
                        }
                    };

                    full_path.push(matched_f);
                }
            }
        }

        MosPath(full_path)
    }

    fn host_path_from_mos_path_join(&mut self, new_fragments: &str) -> std::path::PathBuf {
        let rel_path = self.mos_path_join(new_fragments);
        self.mos_path_to_host_path(&rel_path)
    }

    fn hostfs_mos_f_lseek(&mut self, cpu: &mut Cpu) {
        let fptr = self._peek24(cpu.state.sp() + 3);
        let offset = self._peek24(cpu.state.sp() + 6);

        //eprintln!("f_lseek(${:x}, {})", fptr, offset);

        match self.open_files.get(&fptr) {
            Some(mut f) => {
                match f.seek(SeekFrom::Start(offset as u64)) {
                    Ok(pos) => {
                        // save file position to FIL.fptr
                        self._poke24(fptr + mos::FIL_MEMBER_FPTR, pos as u32);
                        // success
                        cpu.state.reg.set24(Reg16::HL, 0);
                    }
                    Err(_) => {
                        cpu.state.reg.set24(Reg16::HL, 1); // error
                    }
                }
            }
            None => {
                cpu.state.reg.set24(Reg16::HL, 1); // error
            }
        }
        Environment::new(&mut cpu.state, self).subroutine_return();
    }

    fn hostfs_mos_f_stat(&mut self, cpu: &mut Cpu) {
        let path_str = {
            let ptr = self._peek24(cpu.state.sp() + 3);
            unsafe {
                String::from_utf8_unchecked(z80_mem_tools::get_cstring(self, ptr))
            }
        };
        let filinfo_ptr = self._peek24(cpu.state.sp() + 6);
        let path = self.host_path_from_mos_path_join(&path_str);
        //eprintln!("f_stat(\"{}\", ${:x})", path_str, filinfo_ptr);

        match std::fs::metadata(&path) {
            Ok(metadata) => {
                // clear the FILINFO struct
                z80_mem_tools::memset(self, filinfo_ptr, 0, mos::SIZEOF_MOS_FILINFO_STRUCT);

                self.hostfs_set_filinfo_from_metadata(filinfo_ptr, &path, &metadata);

                // success
                cpu.state.reg.set24(Reg16::HL, 0);
            }
            Err(e) => {
                match e.kind() {
                    std::io::ErrorKind::NotFound => {
                        cpu.state.reg.set24(Reg16::HL, 4);
                    }
                    _ => {
                        cpu.state.reg.set24(Reg16::HL, 1);
                    }
                }
            }
        }

        Environment::new(&mut cpu.state, self).subroutine_return();
    }

    fn hostfs_mos_f_open(&mut self, cpu: &mut Cpu) {
        let fptr = self._peek24(cpu.state.sp() + 3);
        let filename = {
            let ptr = self._peek24(cpu.state.sp() + 6);
            // MOS filenames may not be valid utf-8
            unsafe {
                String::from_utf8_unchecked(z80_mem_tools::get_cstring(self, ptr))
            }
        };
        let path = self.mos_path_join(&filename);
        let mode = self._peek24(cpu.state.sp() + 9);
        //eprintln!("f_open(${:x}, \"{}\", {})", fptr, &filename, mode);
        match std::fs::File::options()
            .read(true)
            .write(mode & mos::FA_WRITE != 0)
            .create((mode & mos::FA_CREATE_NEW != 0) || (mode & mos::FA_CREATE_ALWAYS != 0))
            .truncate(mode & mos::FA_CREATE_ALWAYS != 0)
            .open(self.mos_path_to_host_path(&path)) {
            Ok(mut f) => {
                // wipe the FIL structure
                z80_mem_tools::memset(self, fptr, 0, mos::SIZEOF_MOS_FIL_STRUCT);

                // save the size in the FIL structure
                let mut file_len = f.seek(SeekFrom::End(0)).unwrap();
                f.seek(SeekFrom::Start(0)).unwrap();

                // XXX don't support files larger than 512KiB
                file_len = file_len.min(1<<19);

                // store file len in fatfs FIL structure
                self._poke24(fptr + mos::FIL_MEMBER_OBJSIZE, file_len as u32);
                
                // store mapping from MOS *FIL to rust File
                self.open_files.insert(fptr, f);

                cpu.state.reg.set24(Reg16::HL, 0); // ok
            }
            Err(e) => {
                match e.kind() {
                    std::io::ErrorKind::NotFound => cpu.state.reg.set24(Reg16::HL, 4),
                    _ => cpu.state.reg.set24(Reg16::HL, 1)
                }
            }

        }
        Environment::new(&mut cpu.state, self).subroutine_return();
    }

    pub fn get_return_address (&mut self,cpu: &mut Cpu) -> u32
    {
        let mut mb:u32 = (cpu.state.reg.mbase).into();
        mb = mb<<16;
        let sp:u16 = cpu.state.reg.get16(Reg16::SP);
        let full_sp:u32 = mb+(sp as u32);
        let mut retaddr:u16 = (self.peek (full_sp+1) as u16)<<8;
        retaddr = retaddr + (self.peek (full_sp) as u16);
        let full_retaddr:u32 = mb+(retaddr as u32);
        
        return full_retaddr;
    }

    pub fn getfcb (&mut self,cpu: &mut Cpu)
    {
        let mut mb:u32 = (cpu.state.reg.mbase).into();
        mb = mb<<16;
        let de:u16 = cpu.state.reg.get16(Reg16::DE);
        let full_de:u32 = mb+(de as u32);
        let filename = &self.mem[(full_de+1) as usize .. (full_de+9) as usize];
        if let Ok(s) = str::from_utf8(filename) {
            eprint!("{}.", s);
        }
        let ext = &self.mem[(full_de+9) as usize .. (full_de+12) as usize];
        if let Ok(s) = str::from_utf8(ext) {
            eprintln!("{}", s);
        }
    }
    pub fn msx_dos_info (&mut self,cpu: &mut Cpu) {
        // BDOS entry in
        if cpu.state.pc() == 0x656d3 { 
            eprintln! ("BDOS entry: {:02X}",cpu.state.reg.get8(Reg8::C));
        }
        // file open
        if cpu.state.pc() == 0x64462 { 
            eprintln! ("BDOS file open");
            self.getfcb (cpu);
        }            
        // random block read
        if cpu.state.pc() == 0x64A43 || cpu.state.pc() == 0x64BC0 { 
            eprintln! ("BDOS block read");
        }       
        // BDOS entry out
        if cpu.state.pc() == 0x65700 { 
            eprintln! ("BDOS entry out");
        }
        if cpu.state.pc() == 0x60100 { 
            eprintln! ("DOS program started");
            self.msxdos_started = true;
        }
        if self.msxdos_started && cpu.state.pc() == 0x60278 { 
            eprintln! ("relocating msxdos.sys");
        }
        if self.msxdos_started && cpu.state.pc() == 0x6db00 { 
            eprintln! ("running relocated msxdos.sys");
        }
    }

    pub fn get_bios_name (&mut self,address:u16) -> &str {
        return match address {
            0x013e => "RDVDP",
            0x0047 => "WRTVDP",
            0x004a => "RDVRM",
            0x004d => "WRTVRM",
            0x0050 => "SETRD",
            0x0053 => "SETWR",
            0x0093 => "WRTPSG",
            0x0096 => "RDPSG",
            0x0141 => "SNSMAT",
            0x0144 => "PHYDIO",
            0x00A2 => "CHPUT",
            0x00b7 => "BREAKX",
            0x009c => "CHSNS",
            0x009f => "CHGET",
            0x0005 => "DOSCALL",
            _ => "UNKNOWN"
        }
    }

    pub fn msx_bios_info (&mut self,cpu: &mut Cpu) {
        let pc:u32 = cpu.state.pc();
        let pc16:u16 = (pc & 0x00ffff) as u16;
        let full_retaddr:u32 = self.get_return_address (cpu);
            
        if pc>=0x50000 && pc<0x90000 {    
            match pc16 {
                0x01d8 => { 
                    //cpu.set_trace(true); 
                }
                0x000c => {
                    eprint!("{:06X} - RDSLT ({:02X},{:04X}) => ",full_retaddr-3,cpu.state.reg.get8(Reg8::A),cpu.state.reg.get16(Reg16::HL));
                }
                0x0308 => {
                    let value:u8 = cpu.state.reg.get8(Reg8::A);
                    eprintln!("{:02X}",value);
                }
                0x0014 => {
                    eprintln!("{:06X} - WRSLT ({:02X},{:04X},{:02X})",full_retaddr-3,cpu.state.reg.get8(Reg8::A),cpu.state.reg.get16(Reg16::HL),cpu.state.reg.get8(Reg8::E));
                }
                0x001c => {
                    eprintln!("{:06X} - CALSLT ({:02X},{:04X})",full_retaddr-3,cpu.state.reg.get8(Reg8::IYH),cpu.state.reg.get16(Reg16::IX));
                }
                0x0024 => {
                    eprintln!("{:06X} - ENASLT({:02X},{:02X})",full_retaddr-3,cpu.state.reg.get8(Reg8::A),cpu.state.reg.get8(Reg8::H));
                }
                0x0030 => {
                    eprintln!("{:06X} - CALLF ({:02X},{:02X}{:02X})",full_retaddr-3,self.peek(full_retaddr),self.peek(full_retaddr+2),self.peek(full_retaddr+1));
                }
                //0x0028 => {
                //    let port:u8 = self.peek(full_retaddr);
                //    let value:u8 = cpu.state.reg.get8(Reg8::A);
                //    eprintln!("{:06X} - OUT ({:02X}),A <= {:02X}",full_retaddr-1,port,value);
                //}
                0x0248 => {
                    let port:u8 = self.peek(full_retaddr);
                    eprint!("{:06X} - IN A,({:02X}) => ",full_retaddr-1,port);
                }
                /*
                0x0000..=0x3fff => {
                    let addr_hi:u8 = ((pc16>>8)&0xff) as u8;
                    let addr_lo:u8 = (pc16&0xff) as u8;
                    let addr_shi:u8 = self.peek (full_retaddr-1);
                    let addr_slo:u8 = self.peek (full_retaddr-2);
                    let cmd:u8 = self.peek (full_retaddr-3);
                    let callfrom_hi:u8 = (((full_retaddr-3) >> 8) & 0xff) as u8;
                    if callfrom_hi>=0x40 {
                        if addr_shi==addr_hi && addr_slo==addr_lo {
                            eprintln! ("{:06X} - BIOSCALL {:04X}-{}\tA={:02X},BC={:04X},DE={:04X},HL={:04X}",
                                full_retaddr-3,
                                pc16,self.get_bios_name(pc16),
                                cpu.state.reg.get8(Reg8::A),
                                cpu.state.reg.get16(Reg16::BC),
                                cpu.state.reg.get16(Reg16::DE),
                                cpu.state.reg.get16(Reg16::HL)
                            );
                        }
                    }
                }
                */
                _ => {
                    
                }
            }
        }
    }

    pub fn start(&mut self) {
        let mut cpu = Cpu::new_ez80();
        let mut last_vsync_count = 0_u32;

        self.load_mos();

        cpu.state.set_pc(0x0000);

        let mut _trace_for = 0;

        loop {
            //std::thread::sleep(std::time::Duration::from_millis(1));

            // fire uart interrupt
            if cpu.state.instructions_executed % 1024 == 0 && self.maybe_fill_rx_buf() != None {
                let mut env = Environment::new(&mut cpu.state, self);
                env.interrupt(0x18); // uart0_handler
            }
            if cpu.state.instructions_executed % 16 == 0 && self.transmit_interrupt == true
            {
                let mut env = Environment::new(&mut cpu.state, self);
                env.interrupt (0x18); // uart0_handler
            }
            // fire vsync interrupt
            {
                let cur_vsync_count = self.vsync_counter.load(std::sync::atomic::Ordering::Relaxed);
                if cur_vsync_count != last_vsync_count {
                    last_vsync_count = cur_vsync_count;
                    let mut env = Environment::new(&mut cpu.state, self);
                    env.interrupt(0x32);
                }
            }

            if self.enable_hostfs && cpu.state.pc()!=0x0000 {
                if cpu.state.pc() == MOS_103_MAP.f_close { self.hostfs_mos_f_close(&mut cpu); }
                if cpu.state.pc() == MOS_103_MAP.f_gets { self.hostfs_mos_f_gets(&mut cpu); }
                if cpu.state.pc() == MOS_103_MAP.f_read { self.hostfs_mos_f_read(&mut cpu); }
                if cpu.state.pc() == MOS_103_MAP.f_open { self.hostfs_mos_f_open(&mut cpu); }
                if cpu.state.pc() == MOS_103_MAP.f_write { self.hostfs_mos_f_write(&mut cpu); }
                if cpu.state.pc() == MOS_103_MAP.f_chdir { self.hostfs_mos_f_chdir(&mut cpu); }
                if cpu.state.pc() == MOS_103_MAP.f_closedir { self.hostfs_mos_f_closedir(&mut cpu); }
                if cpu.state.pc() == MOS_103_MAP.f_getlabel { self.hostfs_mos_f_getlabel(&mut cpu); }
                if cpu.state.pc() == MOS_103_MAP.f_lseek { self.hostfs_mos_f_lseek(&mut cpu); }
                if cpu.state.pc() == MOS_103_MAP.f_mkdir { self.hostfs_mos_f_mkdir(&mut cpu); }
                if cpu.state.pc() == MOS_103_MAP.f_mount { self.hostfs_mos_f_mount(&mut cpu); }
                if cpu.state.pc() == MOS_103_MAP.f_opendir { self.hostfs_mos_f_opendir(&mut cpu); }
                if cpu.state.pc() == MOS_103_MAP.f_putc { self.hostfs_mos_f_putc(&mut cpu); }
                if cpu.state.pc() == MOS_103_MAP.f_readdir { self.hostfs_mos_f_readdir(&mut cpu); }
                if cpu.state.pc() == MOS_103_MAP.f_rename { self.hostfs_mos_f_rename(&mut cpu); }
                if cpu.state.pc() == MOS_103_MAP.f_stat { self.hostfs_mos_f_stat(&mut cpu); }
                if cpu.state.pc() == MOS_103_MAP.f_unlink { self.hostfs_mos_f_unlink(&mut cpu); }
            }

            if cpu.state.pc()==0x64098 {
                // dummy loop
                let a:u8 = 3;
            } 
            if cpu.state.pc()==0x6401a {
                // tickMain Kings Valley
                //cpu.set_trace (true);
            }
            if cpu.state.pc()==0x6401d {
                // past RDVDP
                let a:u8 = 3;
            }  
            if cpu.state.pc()==0x97C5 {
                // get vsync 
                let a:u8 = 3;
                //cpu.set_trace (true);
            }
            if cpu.state.pc()==0x640cc {
                // konamilogo
                let a:u8 = 3;
                cpu.set_trace (true);
            }
            //if cpu.state.pc()==0x6409d {
            //    // rungame
            //    let a:u8 = 3;
            //    cpu.set_trace (true);
            //}

            self.msx_dos_info (&mut cpu);
            self.msx_bios_info (&mut cpu);

            cpu.execute_instruction(self);
        }
    }
}

// misc Machine tools
mod z80_mem_tools {
    use ez80::Machine;

    pub fn memset<M: Machine>(machine: &mut M, address: u32, fill: u8, count: u32) {
        for loc in address..(address + count) {
            machine.poke(loc, fill);
        }
    }

    pub fn memcpy_to_z80<M: Machine>(machine: &mut M, start: u32, data: &[u8]) {
        let mut loc = start;
        for byte in data {
            machine.poke(loc, *byte);
            loc += 1;
        }
    }

    pub fn get_cstring<M: Machine>(machine: &M, address: u32) -> Vec<u8> {
        let mut s: Vec<u8> = vec![];
        let mut ptr = address;

        loop {
            match machine.peek(ptr) {
                0 => break,
                b => s.push(b)
            }
            ptr += 1;
        }
        s
    }

    pub fn checksum<M: Machine>(machine: &M, start: u32, len: u32) -> u32 {
        let mut checksum = 0u32;
        for i in (start..(start+len)).step_by(3) {
            checksum ^= machine._peek24(i as u32);
        }
        checksum
    }
}
