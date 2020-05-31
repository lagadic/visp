/*
* Simd Library (http://ermig1979.github.io/Simd).
*
* Copyright (c) 2011-2019 Yermalayeu Ihar,
*               2014-2019 Antonenka Mikhail,
*               2019-2019 Artur Voronkov.
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/
#ifndef __SimdFrame_hpp__
#define __SimdFrame_hpp__

#include "Simd/SimdLib.hpp"

namespace Simd
{
    /*! @ingroup cpp_frame

        \short The Frame structure provides storage and manipulation of frames (multiplanar images).

        \ref cpp_frame_functions.
    */
    template <template<class> class A>
    struct Frame
    {
        typedef A<uint8_t> Allocator; /*!< Allocator type definition. */

        /*! Maximal count of pixel planes in a frame. */
        static const size_t PLANE_COUNT_MAX = 4;

        /*!
            \enum Format
            Describes pixel format types of a frame.
        */
        enum Format
        {
            /*! An undefined pixel format. */
            None = 0,
            /*! One plane 32-bit (4 8-bit channels) BGRA (Blue, Green, Red, Alpha) pixel format. */
            Bgra32,
            /*! One plane 24-bit (3 8-bit channels) BGR (Blue, Green, Red) pixel format. */
            Bgr24,
            /*! One plane 8-bit gray pixel format. */
            Gray8,
        };

        const size_t width; /*!< \brief A width of the frame. */
        const size_t height; /*!< \brief A height of the frame. */
        const Format format; /*!< \brief A pixel format types of the frame. */
        View<A> planes[PLANE_COUNT_MAX];/*!< \brief Planes of the frame. */
        bool flipped; /*!< \brief A flag of vertically flipped image (false - frame point (0, 0) is placed at top left corner of the frame, true - frame point (0, 0) is placed at bottom left corner of the frame. */
        double timestamp; /*!< \brief A timestamp of the frame. */

        /*!
            Creates a new empty Frame structure.
        */
        Frame();

        /*!
            Creates a new Frame structure on the base of the other frame.

            \note This constructor is not create new frame! It only creates a reference to the same frame. If you want to create a copy then must use method Simd::FRame::Clone.

            \param [in] frame - an original frame.
        */
        Frame(const Frame & frame);

        /*!
            Creates a new one plane Frame structure on the base of the image view.

            \note This constructor is not create new image frame! It only creates a reference to the same image. If you want to create a copy then must use method Simd::Frame::Clone.

            \param [in] view - an original image view.
            \param [in] flipped_ - a flag of vertically flipped image of created frame. It is equal to false by default.
            \param [in] timestamp_ - a timestamp of created frame. It is equal to 0 by default.
        */
        Frame(const View<A> & view, bool flipped_ = false, double timestamp_ = 0);

        /*!
            Creates a new Frame structure with specified width, height and pixel format.

            \param [in] width_ - a width of created frame.
            \param [in] height_ - a height of created frame.
            \param [in] format_ - a pixel format of created frame.
            \param [in] flipped_ - a flag of vertically flipped image of created frame. It is equal to false by default.
            \param [in] timestamp_ - a timestamp of created frame. It is equal to 0 by default.
        */
        Frame(size_t width_, size_t height_, Format format_, bool flipped_ = false, double timestamp_ = 0);

        /*!
            Creates a new Frame structure with specified width, height and pixel format.

            \param [in] size - a size (width and height) of created frame.
            \param [in] format_ - a pixel format of created frame.
            \param [in] flipped_ - a flag of vertically flipped image of created frame. It is equal to false by default.
            \param [in] timestamp_ - a timestamp of created frame. It is equal to 0 by default.
        */
        Frame(const Point<ptrdiff_t> & size, Format format_, bool flipped_ = false, double timestamp_ = 0);

        /*!
            Creates a new Frame structure with specified width, height and pixel format around external buffers.

            \param [in] width_ - a width of created frame.
            \param [in] height_ - a height of created frame.
            \param [in] format_ - a pixel format of created frame.
            \param [in] data0 - a pointer to the pixel data of first image plane.
            \param [in] stride0 - a row size of first image plane.
            \param [in] data1 - a pointer to the pixel data of second image plane.
            \param [in] stride1 - a row size of second image plane.
            \param [in] data2 - a pointer to the pixel data of third image plane.
            \param [in] stride2 - a row size of third image plane.
            \param [in] flipped_ - a flag of vertically flipped image of created frame. It is equal to false by default.
            \param [in] timestamp_ - a timestamp of created frame. It is equal to 0 by default.
        */
        Frame(size_t width_, size_t height_, Format format_, uint8_t * data0, size_t stride0,
            uint8_t * data1, size_t stride1, uint8_t * data2, size_t stride2, bool flipped_ = false, double timestamp_ = 0);

        /*!
            A Frame destructor.
        */
        ~Frame();

        /*!
            Gets a copy of current frame.

            \return a pointer to the new Frame structure. The user must free this pointer after usage.
        */
        Frame * Clone() const;

        /*!
            Gets a copy of current frame using buffer as a storage.

            \param [in, out] buffer - an external frame as a buffer.
            \return a pointer to the new Frame structure (not owner). The user must free this pointer after usage.
        */
        Frame * Clone(Frame & buffer) const;

        /*!
            Creates reference to other Frame structure.

            \note This function is not create copy of the frame! It only create a reference to the same frame.

            \param [in] frame - an original frame.
            \return a reference to itself.
        */
        Frame & operator = (const Frame & frame);

        /*!
            Creates reference to itself.

            \return a reference to itself.
        */
        Frame & Ref();

        /*!
            Re-creates a Frame structure with specified width, height and pixel format.

            \param [in] width_ - a width of re-created frame.
            \param [in] height_ - a height of re-created frame.
            \param [in] format_ - a pixel format of re-created frame.
        */
        void Recreate(size_t width_, size_t height_, Format format_);

        /*!
            Re-creates a Frame structure with specified width, height and pixel format.

            \param [in] size - a size (width and height) of re-created frame.
            \param [in] format_ - a pixel format of re-created frame.
        */
        void Recreate(const Point<ptrdiff_t> & size, Format format_);

        /*!
            Creates a new Frame structure which points to the region of current frame bounded by the rectangle with specified coordinates.

            \param [in] left - a left side of the region.
            \param [in] top - a top side of the region.
            \param [in] right - a right side of the region.
            \param [in] bottom - a bottom side of the region.
            \return - a new Frame structure which points to the region of frame.
        */
        Frame Region(const ptrdiff_t & left, const ptrdiff_t & top, const ptrdiff_t & right, const ptrdiff_t & bottom) const;

        /*!
            Creates a new Frame structure which points to the region of current frame bounded by the rectangle with specified coordinates.

            \param [in, out] left - a left side of the required region. Returns the left side of the actual region.
            \param [in, out] top - a top side of the required region. Returns the top side of the actual region.
            \param [in, out] right - a right side of the required region. Returns the right side of the actual region.
            \param [in, out] bottom - a bottom side of the required region. Returns the bottom side of the actual region.
            \return - a new Frame structure which points to the region of frame.
        */
        Frame Region(ptrdiff_t & left, ptrdiff_t & top, ptrdiff_t & right, ptrdiff_t & bottom) const;

        /*!
            Creates a new Frame structure which points to the region of frame bounded by the rectangle with specified coordinates.

            \param [in] topLeft - a top-left corner of the region.
            \param [in] bottomRight - a bottom-right corner of the region.
            \return - a new Frame structure which points to the region of frame.
        */
        Frame Region(const Point<ptrdiff_t> & topLeft, const Point<ptrdiff_t> & bottomRight) const;

        /*!
            Creates a new Frame structure which points to the region of frame bounded by the rectangle with specified coordinates.

            \param [in, out] topLeft - a top-left corner of the required region. Returns the top-left corner of the actual region.
            \param [in, out] bottomRight - a bottom-right corner of the required region. Returns the bottom-right corner of the actual region.
            \return - a new Frame structure which points to the region of frame.
        */
        Frame Region(Point<ptrdiff_t> & topLeft, Point<ptrdiff_t> & bottomRight) const;

        /*!
            Creates a new Frame structure which points to the region of frame bounded by the rectangle with specified coordinates.

            \param [in] rect - a rectangle which bound the region.
            \return - a new Frame structure which points to the region of frame.
        */
        Frame Region(const Rectangle<ptrdiff_t> & rect) const;

        /*!
            Creates a new Frame structure which points to the region of frame bounded by the rectangle with specified coordinates.

            \param [in, out] rect - a rectangle which bound the required region. Returns the actual region.
            \return - a new Frame structure which points to the region of frame.
        */
        Frame Region(Rectangle<ptrdiff_t> & rect) const;

        /*!
            Creates a new Frame structure which points to the vertically flipped frame.

            \return - a new Frame structure which points to the flipped frame.
        */
        Frame Flipped() const;

        /*!
            Gets size (width and height) of the frame.

            \return - a new Point structure with frame width and height.
        */
        Point<ptrdiff_t> Size() const;

        /*!
            Gets size in bytes required to store pixel data of current Frame structure.

            \return - a size of data pixels in bytes.
        */
        size_t DataSize() const;

        /*!
            Gets area in pixels of of current Frame structure.

            \return - a area of current Frame in pixels.
        */
        size_t Area() const;

        /*!
            \fn size_t PlaneCount(Format format);

            Gets number of planes in the frame for current pixel format.

            \param [in] format - a pixel format.
            \return - a number of planes.
        */
        static size_t PlaneCount(Format format);

        /*!
            Gets number of planes for current frame.

            \return - a number of planes.
        */
        size_t PlaneCount() const;
    };

    /*! @ingroup cpp_frame_functions

        \fn template <template<class> class A, template<class> class B> bool EqualSize(const Frame<A> & a, const Frame<B> & b);

        Checks two frames on the same size.

        \param [in] a - a first frame.
        \param [in] b - a second frame.
        \return - a result of checking.
    */
    template <template<class> class A, template<class> class B> bool EqualSize(const Frame<A> & a, const Frame<B> & b);

    /*! @ingroup cpp_frame_functions

        \fn template <template<class> class A, template<class> class B> bool Compatible(const Frame<A> & a, const Frame<B> & b);

        Checks two frames on compatibility (the frames must have the same size and pixel format).

        \param [in] a - a first frame.
        \param [in] b - a second frame.
        \return - a result of checking.
    */
    template <template<class> class A, template<class> class B> bool Compatible(const Frame<A> & a, const Frame<B> & b);

    /*! @ingroup cpp_frame_functions

        \fn template <template<class> class A, template<class> class B> void Copy(const Frame<A> & src, Frame<B> & dst);

        \short Copies one frame to another frame.

        The frames must have the same width, height and format.

        \param [in] src - an input frame.
        \param [out] dst - an output frame.
    */
    template <template<class> class A, template<class> class B> void Copy(const Frame<A> & src, Frame<B> & dst);

    /*! @ingroup cpp_frame_functions

        \fn template <template<class> class A> void Convert(const Frame<A> & src, Frame<A> & dst);

        \short Converts one frame to another frame.

        The frames must have the same width and height.

        \param [in] src - an input frame.
        \param [out] dst - an output frame.
    */
    template <template<class> class A> void Convert(const Frame<A> & src, Frame<A> & dst);

    //-------------------------------------------------------------------------

    // struct Frame implementation:

    template <template<class> class A> SIMD_INLINE Frame<A>::Frame()
        : width(0)
        , height(0)
        , format(None)
        , flipped(false)
        , timestamp(0)
    {
    }

    template <template<class> class A> SIMD_INLINE Frame<A>::Frame(const Frame & frame)
        : width(frame.width)
        , height(frame.height)
        , format(frame.format)
        , flipped(frame.flipped)
        , timestamp(frame.timestamp)
    {
        for (size_t i = 0, n = PlaneCount(); i < n; ++i)
            planes[i] = frame.planes[i];
    }

    template <template<class> class A> SIMD_INLINE Frame<A>::Frame(const View<A> & view, bool flipped_, double timestamp_)
        : width(view.width)
        , height(view.height)
        , format(None)
        , flipped(flipped_)
        , timestamp(timestamp_)
    {
        switch (view.format)
        {
        case View<A>::Gray8: (Format&)format = Gray8; break;
        case View<A>::Bgr24: (Format&)format = Bgr24; break;
        case View<A>::Bgra32: (Format&)format = Bgra32; break;
        default:
            assert(0);
        }
        planes[0] = view;
    }

    template <template<class> class A> SIMD_INLINE Frame<A>::Frame(size_t width_, size_t height_, Format format_, bool flipped_, double timestamp_)
        : width(0)
        , height(0)
        , format(None)
        , flipped(flipped_)
        , timestamp(timestamp_)
    {
        Recreate(width_, height_, format_);
    }

    template <template<class> class A> SIMD_INLINE Frame<A>::Frame(const Point<ptrdiff_t> & size, Format format_, bool flipped_, double timestamp_)
        : width(0)
        , height(0)
        , format(None)
        , flipped(flipped_)
        , timestamp(timestamp_)
    {
        Recreate(size, format_);
    }

    template <template<class> class A> SIMD_INLINE Frame<A>::Frame(size_t width_, size_t height_, Format format_, uint8_t * data0, size_t stride0,
        uint8_t * data1, size_t stride1, uint8_t * data2, size_t stride2, bool flipped_, double timestamp_)
        : width(width_)
        , height(height_)
        , format(format_)
        , flipped(flipped_)
        , timestamp(timestamp_)
    {
        switch (format)
        {
        case None:
            break;
        case Bgra32:
            planes[0] = View<A>(width, height, stride0, View<A>::Bgra32, data0);
            break;
        case Bgr24:
            planes[0] = View<A>(width, height, stride0, View<A>::Bgr24, data0);
            break;
        case Gray8:
            planes[0] = View<A>(width, height, stride0, View<A>::Gray8, data0);
            break;
        }
    }

    template <template<class> class A> SIMD_INLINE Frame<A>::~Frame()
    {
    }

    template <template<class> class A> SIMD_INLINE Frame<A> * Frame<A>::Clone() const
    {
        Frame<A> * clone = new Frame<A>(width, height, format, flipped, timestamp);
        Copy(*this, *clone);
        return clone;
    }

    /*! \cond */
    template <template<class> class A> SIMD_INLINE Frame<A> * Frame<A>::Clone(Frame<A> & buffer) const
    {
        for (size_t i = 0; i < PlaneCount(); ++i)
        {
            if (buffer.planes[i].width < planes[i].width || buffer.planes[i].height < planes[i].height)
                buffer.planes[i].Recreate(planes[i].Size(), planes[i].format);
        }
        Frame<A> * clone = new Frame<A>(width, height, format,
                                        buffer.planes[0].data, buffer.planes[0].stride,
                                        buffer.planes[1].data, buffer.planes[1].stride,
                                        buffer.planes[2].data, buffer.planes[2].stride,
                                        flipped, timestamp);
        Copy(*this, *clone);
        return clone;
    }
    /*! \endcond */

    template <template<class> class A> SIMD_INLINE Frame<A> & Frame<A>::operator = (const Frame<A> & frame)
    {
        if (this != &frame)
        {
            *(size_t*)&width = frame.width;
            *(size_t*)&height = frame.height;
            *(Format*)&format = frame.format;
            flipped = frame.flipped;
            timestamp = frame.timestamp;
            for (size_t i = 0, n = PlaneCount(); i < n; ++i)
                planes[i] = frame.planes[i];
        }
        return *this;
    }

    template <template<class> class A> SIMD_INLINE Frame<A> & Frame<A>::Ref()
    {
        return *this;
    }

    template <template<class> class A> SIMD_INLINE void Frame<A>::Recreate(size_t width_, size_t height_, Format format_)
    {
        *(size_t*)&width = width_;
        *(size_t*)&height = height_;
        *(Format*)&format = format_;

        for (size_t i = 0; i < PLANE_COUNT_MAX; ++i)
            planes[i].Recreate(0, 0, View<A>::None);

        switch (format)
        {
        case None:
            break;
        case Bgra32:
            planes[0].Recreate(width, height, View<A>::Bgra32);
            break;
        case Bgr24:
            planes[0].Recreate(width, height, View<A>::Bgr24);
            break;
        case Gray8:
            planes[0].Recreate(width, height, View<A>::Gray8);
            break;
        }
    }

    template <template<class> class A> SIMD_INLINE void Frame<A>::Recreate(const Point<ptrdiff_t> & size, Format format_)
    {
        Recreate(size.x, size.y, format_);
    }

    template <template<class> class A> SIMD_INLINE Frame<A> Frame<A>::Region(const ptrdiff_t & left, const ptrdiff_t & top, const ptrdiff_t & right, const ptrdiff_t & bottom) const
    {
        Rectangle<ptrdiff_t> rect(left, top, right, bottom);
        return Region(rect.left, rect.top, rect.right, rect.bottom);
    }

    template <template<class> class A> SIMD_INLINE Frame<A> Frame<A>::Region(ptrdiff_t & left, ptrdiff_t & top, ptrdiff_t & right, ptrdiff_t & bottom) const
    {
        if (format != None && right >= left && bottom >= top)
        {
            left = std::min<ptrdiff_t>(std::max<ptrdiff_t>(left, 0), width);
            top = std::min<ptrdiff_t>(std::max<ptrdiff_t>(top, 0), height);
            right = std::min<ptrdiff_t>(std::max<ptrdiff_t>(right, 0), width);
            bottom = std::min<ptrdiff_t>(std::max<ptrdiff_t>(bottom, 0), height);

            Frame frame;
            *(size_t*)&frame.width = right - left;
            *(size_t*)&frame.height = bottom - top;
            *(Format*)&frame.format = format;
            frame.flipped = flipped;
            frame.timestamp = timestamp;

            frame.planes[0] = planes[0].Region(left, top, right, bottom);

            return frame;
        }
        else
            return Frame<A>();
    }

    template <template<class> class A> SIMD_INLINE Frame<A> Frame<A>::Region(const Point<ptrdiff_t> & topLeft, const Point<ptrdiff_t> & bottomRight) const
    {
        return Region(topLeft.x, topLeft.y, bottomRight.x, bottomRight.y);
    }

    template <template<class> class A> SIMD_INLINE Frame<A> Frame<A>::Region(Point<ptrdiff_t> & topLeft, Point<ptrdiff_t> & bottomRight) const
    {
        return Region(topLeft.x, topLeft.y, bottomRight.x, bottomRight.y);
    }

    template <template<class> class A> SIMD_INLINE Frame<A> Frame<A>::Region(const Rectangle<ptrdiff_t> & rect) const
    {
        return Region(rect.left, rect.top, rect.right, rect.bottom);
    }

    template <template<class> class A> SIMD_INLINE Frame<A> Frame<A>::Region(Rectangle<ptrdiff_t> & rect) const
    {
        return Region(rect.left, rect.top, rect.right, rect.bottom);
    }

    template <template<class> class A> SIMD_INLINE Frame<A> Frame<A>::Flipped() const
    {
        Frame frame;
        *(size_t*)&frame.width = width;
        *(size_t*)&frame.height = height;
        *(Format*)&frame.format = format;
        frame.timestamp = timestamp;
        frame.flipped = !flipped;
        for (size_t i = 0, n = PlaneCount(); i < n; ++i)
            frame.planes[i] = planes[i].Flipped();
        return frame;
    }

    template <template<class> class A> SIMD_INLINE Point<ptrdiff_t> Frame<A>::Size() const
    {
        return Point<ptrdiff_t>(width, height);
    }

    template <template<class> class A> SIMD_INLINE size_t Frame<A>::DataSize() const
    {
        size_t size = 0;
        for (size_t i = 0; i < PLANE_COUNT_MAX; ++i)
            size += planes[i].DataSize();
        return size;
    }

    template <template<class> class A> SIMD_INLINE size_t Frame<A>::Area() const
    {
        return width*height;
    }

    template <template<class> class A> SIMD_INLINE size_t Frame<A>::PlaneCount(Format format)
    {
        switch (format)
        {
        case None:    return 0;
        case Bgra32:  return 1;
        case Bgr24:   return 1;
        case Gray8:   return 1;
        default: assert(0); return 0;
        }
    }

    template <template<class> class A> SIMD_INLINE size_t Frame<A>::PlaneCount() const
    {
        return PlaneCount(format);
    }

    // View utilities implementation:

    template <template<class> class A, template<class> class B> SIMD_INLINE bool EqualSize(const Frame<A> & a, const Frame<B> & b)
    {
        return
            (a.width == b.width && a.height == b.height);
    }

    template <template<class> class A, template<class> class B> SIMD_INLINE bool Compatible(const Frame<A> & a, const Frame<B> & b)
    {
        typedef typename Frame<A>::Format Format;

        return
            (a.width == b.width && a.height == b.height && a.format == (Format)b.format && a.flipped == b.flipped);
    }

    template <template<class> class A, template<class> class B> SIMD_INLINE void Copy(const Frame<A> & src, Frame<B> & dst)
    {
        assert(Compatible(src, dst));

        if (src.format)
        {
            for (size_t i = 0, n = src.PlaneCount(); i < n; ++i)
                Simd::Copy(src.planes[i], dst.planes[i]);
        }
    }

    template <template<class> class A> SIMD_INLINE void Convert(const Frame<A> & src, Frame<A> & dst)
    {
        assert(EqualSize(src, dst) && src.format && dst.format && src.flipped == dst.flipped);

        if (src.format == dst.format)
        {
            Copy(src, dst);
            return;
        }

        switch (src.format)
        {
        case Frame<A>::Bgra32:
            switch (dst.format)
            {
            case Frame<A>::Bgr24:
                BgraToBgr(src.planes[0], dst.planes[0]);
                break;
            case Frame<A>::Gray8:
                BgraToGray(src.planes[0], dst.planes[0]);
                break;
            default:
                assert(0);
            }
            break;

        case Frame<A>::Bgr24:
            switch (dst.format)
            {
            case Frame<A>::Bgra32:
                BgrToBgra(src.planes[0], dst.planes[0]);
                break;
            case Frame<A>::Gray8:
                BgrToGray(src.planes[0], dst.planes[0]);
                break;
            default:
                assert(0);
            }
            break;

        case Frame<A>::Gray8:
            switch (dst.format)
            {
            case Frame<A>::Bgra32:
                GrayToBgra(src.planes[0], dst.planes[0]);
                break;
            case Frame<A>::Bgr24:
                GrayToBgr(src.planes[0], dst.planes[0]);
                break;
            default:
                assert(0);
            }
            break;

        default:
            assert(0);
        }
    }
}

#endif//__SimdFrame_hpp__
