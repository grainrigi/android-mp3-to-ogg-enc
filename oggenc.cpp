#include <iostream>
#include <string.h>
#include <vorbis/vorbisenc.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "Mp3Decoder.h"

using namespace std;

static int last_outsize = 0;
static int total_samples = 0;

void print_progress(int samples)
{
	fwrite(&samples, 4, 1, stdout);
}


constexpr int READ = 1024;
constexpr int ERR_COMMAND_ILLEGAL = -1;
constexpr int ERR_OGG_NOTFOUND = -2;
constexpr int ERR_MP3_UNLOADABLE = -3;
signed char readbuffer[READ * 4 + 44];

int main(int argc, char *argv[]){
	ogg_stream_state os;
	ogg_page og;
	ogg_packet op;
	int ret, eos = 0;
	int i, founddata;

	vorbis_info vi;
	vorbis_comment vc;
	vorbis_dsp_state vd;
	vorbis_block vb;

	void *file_data;
	int bytes_left = 0;
	int byte_count = 0;


	FILE *fp_ogg;
	int fd_mp3;
	char *file_ogg, *file_mp3;

	/* Get the filename from cmdline */
	if(argc < 3)
	{
		std::cerr << "usage: oggenc [mp3 filename] [ogg filename]" << std::endl;
		//int errcode = -1;
		return 1;
	}
	file_ogg = argv[2];
	file_mp3 = argv[1];

	/* Initialize Mp3 Decoding */
	fd_mp3 = open(file_mp3, O_RDONLY);
	if(fd_mp3 < 0)
	{
		std::cerr << "Could not open " << file_mp3 << std::endl;
		return 1;
	}

	bytes_left = lseek(fd_mp3, 0, SEEK_END);
	file_data = mmap(0, bytes_left, PROT_READ, MAP_PRIVATE, fd_mp3, 0);
	Mp3Decoder dec(file_data, bytes_left);

	total_samples = dec.estimate_total_pcm_size();
	
	/* Initialize Ogg Encoding */

	fp_ogg = fopen(file_ogg, "wb");
	if(fp_ogg == NULL)
	{
		std::cerr << "Could not open " << file_ogg << std::endl;
		return 1;
	}

	vorbis_info_init(&vi);
	ret = vorbis_encode_init_vbr(&vi, 2, 44100, .4);

	if(ret)
	{
		std::cerr << "vorbis_encode_init_vbr failed" << std::endl;
		return 1;
	}

	vorbis_comment_init(&vc);
	vorbis_comment_add_tag(&vc, "ENCODER", "ICSSdroid oggenc");

	//set up the analysis state and auxiliary encoding storage
	vorbis_analysis_init(&vd, &vi);
	vorbis_block_init(&vd, &vb);

	//set up our packet->stream encoder
	//pick a random serial number
	srand(time(NULL));
	ogg_stream_init(&os, rand());

	{
		ogg_packet header;
		ogg_packet header_comm;
		ogg_packet header_code;

		vorbis_analysis_headerout(&vd, &vc, &header, &header_comm, &header_code);
		ogg_stream_packetin(&os, &header);
		ogg_stream_packetin(&os, &header_comm);
		ogg_stream_packetin(&os, &header_code);

		//ensures the actual audio data will start on a new page
		while(!eos)
		{
			ret = ogg_stream_flush(&os, &og);
			if(ret == 0) break;
			fwrite(og.header, 1, og.header_len, fp_ogg);
			fwrite(og.body, 1, og.body_len, fp_ogg);
		}
	}
	constexpr float div_32768f = 1.0f / 32768.f;
	int proceeded_samples = 0;

	while(!eos)
	{
		long i;
		long bytes = dec.decode(readbuffer, READ * 4);

		print_progress(proceeded_samples);
		proceeded_samples += bytes;

		if(bytes == 0){
			//End of file
			vorbis_analysis_wrote(&vd, 0);
		}
		else{
			float **buffer = vorbis_analysis_buffer(&vd, READ);

			for(i = 0; i < bytes / 4; i++){
				buffer[0][i]=((readbuffer[i*4+1]<<8)|
						(0x00ff&(int)readbuffer[i*4])) * div_32768f;
				buffer[1][i]=((readbuffer[i*4+3]<<8)|
						(0x00ff&(int)readbuffer[i*4+2])) * div_32768f;	
			}

			vorbis_analysis_wrote(&vd, i);
		}

		while(vorbis_analysis_blockout(&vd, &vb) == 1){
			vorbis_analysis(&vb, NULL);
			vorbis_bitrate_addblock(&vb);

			while(vorbis_bitrate_flushpacket(&vd, &op)){
				ogg_stream_packetin(&os, &op);

				while(!eos){
					ret = ogg_stream_pageout(&os, &og);
					if(ret == 0) break;
					fwrite(og.header, 1, og.header_len, fp_ogg);
					fwrite(og.body, 1, og.body_len, fp_ogg);

					if(ogg_page_eos(&og))eos = 1;
				}
			}
		}
	}

	fclose(fp_ogg);
	close(fd_mp3);

	ogg_stream_clear(&os);
	vorbis_block_clear(&vb);
	vorbis_dsp_clear(&vd);
	vorbis_comment_clear(&vc);
	vorbis_info_clear(&vi);

	std::cout << std::endl;
	std::cout << "Done." << std::endl;
	
	return 0;
}
