#ifndef ColorSchemer_H
#define ColorSchemer_H
//---------------------------------------------------------------------------
#include "CommonHdrXWu.h"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

const Vector3f color_list[] = {
    makeVector3f(224, 7, 7) / 255,
    makeVector3f(20, 133, 204) / 255,
    makeVector3f(255, 177, 0) / 255,
    makeVector3f(255, 252, 25) / 255, // yellow
    makeVector3f(25, 255, 162) / 255,
    makeVector3f(159, 20, 204) / 255,
    makeVector3f(119, 0, 60) / 255,
    makeVector3f(20, 182, 204) / 255,
    makeVector3f(178, 173, 9) / 255,
    makeVector3f(0, 86, 130) / 255,
    makeVector3f(204, 83, 20) / 255,
    makeVector3f(74, 201, 37) / 255,
    makeVector3f(181, 54, 218) / 255,
    makeVector3f(0, 130, 130) / 255,
    makeVector3f(255, 25, 194) / 255,
    makeVector3f(0, 255, 255) / 255
};
const size_t num_color = sizeof(color_list) / sizeof(color_list[0]);

class ColorSchemer
{
public:
    typedef boost::shared_ptr< ColorSchemer > Ptr;
    typedef boost::shared_ptr< const ColorSchemer > ConstPtr;

public:
    static Vector3f GetColor(unsigned ii) { return color_list[ii % num_color]; }

    static Vector3f GetJetColor(float value)
    {
        if (0 > value) value = 0;
        if (1 < value) value = 1;
        float r, g, b;
        if (0.125 > value) {
            r = g = 0;
            b = value / 0.25 + 0.5f;
        }
        else if (0.375 > value) {
            r = 0; b = 1;
            g = (value - 0.125) / 0.25;
        }
        else if (0.625 > value) {
            g = 1;
            r = (value - 0.375) / 0.25;
            b = 1 - r;
        }
        else if (0.875 > value) {
            r = 1; b = 0;
            g = 1 - (value - 0.625) / 0.25;
        }
        else {
            g = b = 0;
            r = 1 - (value - 0.875) / 0.25;
        }
        return makeVector3f(r, g, b);
    }
    static Vector3f GetJetColor(unsigned value, const unsigned& maxValue)
    {
        return ColorSchemer::GetJetColor((float)value / (float)(maxValue - 1));
    }

    // r,g,b values are from 0 to 1
    // h = [0,360], s = [0,1], v = [0,1]
    //		if s == 0, then h = -1 (undefined)
    static void RGBtoHSV( float r, float g, float b, float *h, float *s, float *v )
    {
        float min, max, delta;
        min = std::min( std::min( r, g ) , b );
        max = std::max( std::max( r, g ) , b );
        *v = max;				// v
        delta = max - min;
        if( max != 0 )
            *s = delta / max;		// s
        else {
            // r = g = b = 0		// s = 0, v is undefined
            *s = 0;
            *h = -1;
            return;
        }
        if( r == max )
            *h = ( g - b ) / delta;		// between yellow & magenta
        else if( g == max )
            *h = 2 + ( b - r ) / delta;	// between cyan & yellow
        else
            *h = 4 + ( r - g ) / delta;	// between magenta & cyan
        *h *= 60;				// degrees
        if( *h < 0 )
            *h += 360;
    }
    static void HSVtoRGB( float *r, float *g, float *b, float h, float s, float v )
    {
        int i;
        float f, p, q, t;
        if( s == 0 ) {
            // achromatic (grey)
            *r = *g = *b = v;
            return;
        }
        h /= 60;			// sector 0 to 5
        i = floor( h );
        f = h - i;			// factorial part of h
        p = v * ( 1 - s );
        q = v * ( 1 - s * f );
        t = v * ( 1 - s * ( 1 - f ) );
        switch( i ) {
        case 0:
            *r = v;
            *g = t;
            *b = p;
            break;
        case 1:
            *r = q;
            *g = v;
            *b = p;
            break;
        case 2:
            *r = p;
            *g = v;
            *b = t;
            break;
        case 3:
            *r = p;
            *g = q;
            *b = v;
            break;
        case 4:
            *r = t;
            *g = p;
            *b = v;
            break;
        default:		// case 5:
            *r = v;
            *g = p;
            *b = q;
            break;
        }
    }
};

template< unsigned ratio = 100 >
class StandardColors
{
public:
    static const Vector3f color_grey;
    static const Vector3f color_red;
    static const Vector3f color_green;
    static const Vector3f color_blue;
    static const Vector3f color_yellow;
    static const Vector3f color_cyan;
    static const Vector3f color_magenta;

private:
    static const float value;
};

template< unsigned ratio > const float StandardColors<ratio>::value = static_cast<float>(ratio) / 100.f;
template< unsigned ratio > const Vector3f StandardColors<ratio>::color_grey       = makeVector3f(value, value, value);
template< unsigned ratio > const Vector3f StandardColors<ratio>::color_red        = makeVector3f(value, 0, 0);
template< unsigned ratio > const Vector3f StandardColors<ratio>::color_green      = makeVector3f(0, value, 0);
template< unsigned ratio > const Vector3f StandardColors<ratio>::color_blue       = makeVector3f(0, 0, value);
template< unsigned ratio > const Vector3f StandardColors<ratio>::color_yellow     = makeVector3f(value, value, 0);
template< unsigned ratio > const Vector3f StandardColors<ratio>::color_cyan       = makeVector3f(0, value, value);
template< unsigned ratio > const Vector3f StandardColors<ratio>::color_magenta    = makeVector3f(value, 0, value);

#endif
