#ifndef __ARCH_X86_DECODER_STRUCTS__
#define __ARCH_X86_DECODER_STRUCTS__

#include "arch/x86/types.hh"
#include "base/types.hh"

struct FullUopAddr {
	Addr pcAddr;
	unsigned uopAddr;

	FullUopAddr(Addr p, unsigned u) {
		pcAddr = p;
		uopAddr = u;
	}

	FullUopAddr() {
		pcAddr = 0;
		uopAddr = 0;
	}

	bool operator==(const FullUopAddr& rhs) {
		return (pcAddr == rhs.pcAddr) && (uopAddr == rhs.uopAddr);
	}

	bool operator!=(const FullUopAddr& rhs) {
		return (pcAddr != rhs.pcAddr) || (uopAddr != rhs.uopAddr);
	}
};

struct FullCacheIdx {
	int idx;
	int way;
	int uop;
	bool valid;

	FullCacheIdx(int i, int w, int u) {
		idx = i;
		way = w;
		uop = u;
		valid = true;
	}

	FullCacheIdx() {
		idx = 0;
		way = 0;
		uop = 0;
		valid = false;
	}

	bool operator==(const FullCacheIdx& rhs) {
		return (idx == rhs.idx) && (way == rhs.way) && (uop == rhs.uop);
	}

	bool operator!=(const FullCacheIdx& rhs) {
		return (idx != rhs.idx) || (way != rhs.way) || (uop != rhs.uop);
	}
};

#endif // __ARCH_X86_DECODER_STRUCTS__
