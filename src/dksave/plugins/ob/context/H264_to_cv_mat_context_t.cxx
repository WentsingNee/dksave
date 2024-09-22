/**
 * @file       H264_to_cv_mat_context_t.cxx
 * @brief
 * @date       2023-10-14
 * @author     Wentsing Nee
 * @copyright
 *      Wentsing Nee of China Agricultural University
 *   all rights reserved
 *
 *      Implement of this file partially references the code: [](https://github.com/hirorogithub/ffmpeg_sample-H264_to_cv-Mat)
 */

module;

#include "kerbal/container/vector.hpp"

#include <memory>

#include <cstdint>
#include <cstdlib>

// Opencv
#include <opencv2/core/core.hpp>

extern "C" {
#include <libswscale/swscale.h>
#include <libavutil/avutil.h>
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
};


export module dksave.plugins.ob.context.H264_to_cv_mat_context_t;


namespace dksave::plugins_ob
{

	struct AVCodecContextDeleter
	{
			void operator()(AVCodecContext * p) noexcept
			{
				avcodec_free_context(&p);
			}
	};

	struct AVPacketDeleter
	{
			void operator()(AVPacket * p) noexcept
			{
				av_packet_free(&p);
			}
	};

	struct AVFrameDeleter
	{
			void operator()(AVFrame * p) noexcept
			{
				av_frame_free(&p);
			}
	};

	struct SwsContextDeleter
	{
			void operator()(SwsContext * p) noexcept
			{
				sws_freeContext(p);
			}
	};


	class H264_bad_alloc : std::bad_alloc
	{
		private:
			using super = std::runtime_error;
			std::string msg;

		public:
			H264_bad_alloc(std::string const & msg) :
				msg(msg)
			{
			}

			virtual
			char const * what() const noexcept
			{
				return msg.c_str();
			}
	};

	export
	class H264_decode_error : std::runtime_error
	{
		private:
			using super = std::runtime_error;

		public:
			H264_decode_error(std::string const & msg) :
				super(msg)
			{
			}
	};


	template <typename T>
	struct av_allocator;

	template <>
	struct av_allocator<void>
	{
			typedef void value_type;
			typedef std::size_t size_type;

			static
			value_type * allocate(size_type size)
			{
				value_type * p = av_malloc(size);
				if (nullptr == p) {
					throw H264_bad_alloc("av_malloc failed");
				}
				return p;
			}

			static
			void deallocate(value_type * p, size_type) noexcept
			{
				av_free(p);
			}
	};

	template <typename T>
	struct av_allocator
	{
		private:
			typedef av_allocator<void> upstream_allocator;

		public:
			typedef T value_type;
			typedef upstream_allocator::size_type size_type;

			static
			value_type * allocate(size_type n)
			{
				return static_cast<value_type *>(
					upstream_allocator::allocate(n * sizeof(value_type))
				);
			}

			static
			void deallocate(value_type * p, size_type n) noexcept
			{
				upstream_allocator::deallocate(p, n * sizeof(value_type));
			}
	};

	export
	class H264_to_cv_mat_context_t
	{

		public :
			H264_to_cv_mat_context_t();

			cv::Mat const & decode(unsigned char * input_buff, size_t size);

		private:
			const AVCodec * av_codec;
			std::unique_ptr<AVCodecContext, AVCodecContextDeleter> av_codec_context;
			std::unique_ptr<AVPacket, AVPacketDeleter> av_packet;
			std::unique_ptr<AVFrame, AVFrameDeleter> av_frame;
			std::unique_ptr<AVFrame, AVFrameDeleter> av_frame_bgr; //存储解码后转换的RGB数据

			kerbal::container::vector<std::uint8_t, av_allocator<std::uint8_t> > in_buffer;
			kerbal::container::vector<std::uint8_t, av_allocator<std::uint8_t> > out_buffer;

			cv::Mat cv_mat;
	};

	H264_to_cv_mat_context_t::H264_to_cv_mat_context_t()
	{
		this->av_codec = avcodec_find_decoder(AV_CODEC_ID_H264);
		if (nullptr == this->av_codec) {
			throw H264_decode_error("Codec not found");
		}

		{
			AVCodecContext * av_codec_context = avcodec_alloc_context3(this->av_codec);
			if (nullptr == av_codec_context) {
				throw H264_bad_alloc("Could not allocate video av_codec context");
			}
			this->av_codec_context.reset(av_codec_context);
		}

		if (avcodec_open2(this->av_codec_context.get(), this->av_codec, nullptr) < 0) {
			throw H264_decode_error("Could not open av_codec");
		}

		{
			AVPacket * av_packet = av_packet_alloc();
			if (nullptr == av_packet) {
				throw H264_bad_alloc("Could not allocate av_packet");
			}
			this->av_packet.reset(av_packet);
		}

		{
			AVFrame * av_frame = av_frame_alloc();
			if (nullptr == av_frame) {
				throw H264_bad_alloc("Could not allocate video av_frame");
			}
			this->av_frame.reset(av_frame);
		}

		{
			AVFrame * av_frame_bgr = av_frame_alloc();
			if (nullptr == av_frame_bgr) {
				throw H264_bad_alloc("Could not allocate av_frame_bgr");
			}
			this->av_frame_bgr.reset(av_frame_bgr);
		}

	}


	cv::Mat const &
	H264_to_cv_mat_context_t::decode(std::uint8_t * input_buff, size_t size)
	{
		if (0 == size) {
			throw H264_decode_error("Size is zero");
		}
		//	this->in_buffer.insert(this->in_buffer.cend(), input_buff, input_buff + size);
		//	this->av_packet->data = &this->in_buffer[0];
		//	this->av_packet->size = this->in_buffer.size();

		this->av_packet->data = input_buff;
		this->av_packet->size = size;

		int send_ret = avcodec_send_packet(this->av_codec_context.get(), this->av_packet.get());
		if (0 != send_ret) {
			throw H264_decode_error("Send packet failed");
		}
		int rece_ret = avcodec_receive_frame(this->av_codec_context.get(), this->av_frame.get());
		if (0 != rece_ret) {
			if (AVERROR(EAGAIN)) {
				throw H264_decode_error("output is not available in this state - user must try to send new input");
			}
			if (AVERROR_EOF) {
				throw H264_decode_error("the codec has been fully flushed, and there will be no more output frames");
			}
			if (AVERROR(EINVAL)) {
				throw H264_decode_error("codec not opened, or it is an encoder without the AV_CODEC_FLAG_RECON_FRAME flag enabled");
			}
			throw H264_decode_error("legitimate decoding errors");
		}

		std::unique_ptr<SwsContext, SwsContextDeleter> sws_context;
		{
			SwsContext * p_sws_context = sws_getContext(
				av_codec_context->width, av_codec_context->height, av_codec_context->pix_fmt,
				av_codec_context->width, av_codec_context->height, AV_PIX_FMT_BGR24, SWS_BICUBIC, nullptr, nullptr,
				nullptr
			);
			if (nullptr == p_sws_context) {
				throw H264_decode_error("Could not allocate sws_context");
			}
			sws_context.reset(p_sws_context);
		}

		int BGR_size = av_image_get_buffer_size(
			AV_PIX_FMT_BGR24,
			av_codec_context->width,
			av_codec_context->height,
			1
		);
		out_buffer.resize(BGR_size);
		if (
			av_image_fill_arrays(
				av_frame_bgr->data, av_frame_bgr->linesize, &out_buffer[0], AV_PIX_FMT_BGR24,
				av_codec_context->width, av_codec_context->height, 1
			) < 0
		) {
			throw H264_decode_error("av_image_fill_arrays failed");
		}

		sws_scale(
			sws_context.get(),
			(const uint8_t * const *) av_frame->data,
			av_frame->linesize,
			0,
			av_codec_context->height,
			av_frame_bgr->data,
			av_frame_bgr->linesize
		);

		this->cv_mat = cv::Mat(
			cv::Size(av_codec_context->width, av_codec_context->height),
			CV_8UC3,
			&out_buffer[0]
		);
		this->in_buffer.clear();
		return this->cv_mat;
	}

} // namespace dksave::plugins_ob
