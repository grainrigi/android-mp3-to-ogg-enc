#pragma once
#include "minimp3.h"
#include "libc.h"
#include <memory>
#include <cmath>
#include <assert.h>
#include <vector>

#define MP3_FRAME_SIZE 1152
#define MP3_MAX_CODED_FRAME_SIZE 1792
#define MP3_MAX_CHANNELS 2
#define MP3_MAX_FRAME_SIZE (MP3_MAX_SAMPLES_PER_FRAME * sizeof(uint16_t) * 2)
#define SBLIMIT 32

#define MP3_STEREO  0
#define MP3_JSTEREO 1
#define MP3_DUAL    2
#define MP3_MONO    3

#define SAME_HEADER_MASK \
   (0xffe00000 | (3 << 17) | (0xf << 12) | (3 << 10) | (3 << 19))

#define FRAC_BITS   15
#define WFRAC_BITS  14

#define OUT_MAX (32767)
#define OUT_MIN (-32768)
#define OUT_SHIFT (WFRAC_BITS + FRAC_BITS - 15)

#define MODE_EXT_MS_STEREO 2
#define MODE_EXT_I_STEREO  1

#define FRAC_ONE    (1 << FRAC_BITS)
#define FIX(a)   ((int)((a) * FRAC_ONE))
#define FIXR(a)   ((int)((a) * FRAC_ONE + 0.5))
#define FRAC_RND(a) (((a) + (FRAC_ONE/2)) >> FRAC_BITS)
#define FIXHR(a) ((int)((a) * (1LL<<32) + 0.5))

#define ASM_X86

#if !defined _MSC_VER || (!defined ASM_X86 && !defined ASM_ARM)
#define MULL(a,b) (((int64_t)(a) * (int64_t)(b)) >> FRAC_BITS)
#define MULH(a,b) (((int64_t)(a) * (int64_t)(b)) >> 32)
#else
static INLINE int MULL(int a, int b) {
	int res;
	__asm {
		mov eax, a
		imul b
		shr eax, 15
		shl edx, 17
		or eax, edx
		mov res, eax
	}
	return res;
}
static INLINE int MULH(int a, int b) {
	int res;
	__asm {
		mov eax, a
		imul b
		mov res, edx
	}
	return res;
}
#endif
#define MULS(ra, rb) ((ra) * (rb))

#define ISQRT2 FIXR(0.70710678118654752440)

#define HEADER_SIZE 4
#define BACKSTEP_SIZE 512
#define EXTRABYTES 24

#define VLC_TYPE int16_t

typedef struct _bitstream {
	const uint8_t *buffer, *buffer_end;
	int index;
	int size_in_bits;
} bitstream_t;

typedef struct _vlc {
	int bits;
	VLC_TYPE (*table)[2]; ///< code, bits
	int table_size, table_allocated;
} vlc_t;

typedef struct _huff_table {
	int xsize;
	const uint8_t *bits;
	const uint16_t *codes;
} huff_table_t;

class Mp3Decoder
{
	struct _granule;

	typedef struct _mp3_context {
		uint8_t last_buf[2 * BACKSTEP_SIZE + EXTRABYTES];
		int last_buf_size;
		int frame_size;
		uint32_t free_format_next_header;
		int error_protection;
		int sample_rate;
		int sample_rate_index;
		int bit_rate;
		bitstream_t gb;
		bitstream_t in_gb;
		int nb_channels;
		int mode;
		int mode_ext;
		int lsf;
		int16_t synth_buf[MP3_MAX_CHANNELS][512 * 2];
		int synth_buf_offset[MP3_MAX_CHANNELS];
		int32_t sb_samples[MP3_MAX_CHANNELS][36][SBLIMIT];
		int32_t mdct_buf[MP3_MAX_CHANNELS][SBLIMIT * 18];
		int dither_state;
	} mp3_context_t;

	typedef struct _granule {
		uint8_t scfsi;
		int part2_3_length;
		int big_values;
		int global_gain;
		int scalefac_compress;
		uint8_t block_type;
		uint8_t switch_point;
		int table_select[3];
		int subblock_gain[3];
		uint8_t scalefac_scale;
		uint8_t count1table_select;
		int region_size[3];
		int preflag;
		int short_start, long_end;
		uint8_t scale_factors[40];
		int32_t sb_hybrid[SBLIMIT * 18];
	} granule_t;

	mp3_decoder_t m_mp3;
	void *m_file;
	void *m_firstframe;
	void *m_current_frame;
	int m_filesize;
	int m_bytesleft;
	int m_sample_rate;
	int m_channels;

	class DecodedQueue{
		int frame_size;
		std::unique_ptr<uint8_t> m_buf;
		int pos_start;
		int pos_end;
		int bufsize;
	public:
		DecodedQueue(int max_push_size);

		uint8_t *GetCurrentPushPosition(void);
		int NotifyPushedData(int size);
		int PushData(const void *buf, int size);

		int PopData(void *buf, int size);

		int size(void) const { return pos_start > pos_end ? (pos_end + (bufsize - pos_start)) : pos_end - pos_start; }
	};
	DecodedQueue m_decoded;

	std::vector<std::tuple<int, void*>> m_seek_pivot;
public:
	static vlc_t huff_vlc[16];
	static vlc_t huff_quad_vlc[2];
	static uint16_t band_index_long[9][23];
#define TABLE_4_3_SIZE (8191 + 16)*4
	static int8_t  *table_4_3_exp;
	static uint32_t *table_4_3_value;
	static uint32_t exp_table[512];
	static uint32_t expval_table[512][16];
	static int32_t is_table[2][16];
	static int32_t is_table_lsf[2][2][16];
	static int32_t csa_table[8][4];
	static float csa_table_float[8][4];
	static int32_t mdct_win[8][36];
	static int16_t window[512];

	Mp3Decoder(void *file, int size);
	~Mp3Decoder(void);

	int framesize(void);

	int decode(void *out, int size);
	float seek(float second);
	int samplerate(void)  { return m_sample_rate; }

	int estimate_total_pcm_size(void);

private:
	mp3_decoder_t mp3_create(void);
	int mp3_decode_internal(void *out);
	int mp3_decode(void *buf, int bytes, signed short *out, mp3_info_t *info);
	void mp3_done();
	static bool isDecodeReady;
	static void decode_init(void);
	static int mp3_decode_init(mp3_context_t *s);
	void switch_buffer(mp3_context_t * s, int * pos, int * end_pos, int * end_pos2);
	void exponents_from_scale_factors(
		mp3_context_t *s, granule_t *g, int16_t *exponents
	);
	void reorder_block(mp3_context_t *s, granule_t *g);
	void compute_antialias(mp3_context_t *s, granule_t *g);
	void compute_stereo(
		mp3_context_t *s, granule_t *g0, granule_t *g1
	);
	int huffman_decode(
		mp3_context_t *s, granule_t *g, int16_t *exponents, int end_pos2
	);
	void compute_imdct(
		mp3_context_t *s, granule_t *g, int32_t *sb_samples, int32_t *mdct_buf
	);
	int decode_header(mp3_context_t *s, uint32_t header);
	int mp_decode_layer3(mp3_context_t *s);
	int mp3_decode_main(
		mp3_context_t *s,
		int16_t *samples, const uint8_t *buf, int buf_size
	);
	int mp3_decode_frame(
		mp3_context_t *s,
		int16_t *out_samples, int *data_size,
		uint8_t *buf, int buf_size
	);
	int seek_to_sample(
		int sample
	);
};
