#ifndef GroupOrbit_H
#define GroupOrbit_H
//---------------------------------------------------------------------------
#include "CommonHdrXWu.h"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

class PROJECTSXWU_API GroupOrbit
{
public:
    typedef boost::shared_ptr< GroupOrbit > Ptr;
    typedef boost::shared_ptr< const GroupOrbit > ConstPtr;

public:
    virtual ~GroupOrbit() {}

public:
};

class OrbitCircle : public GroupOrbit
{
public:
    typedef boost::shared_ptr< OrbitCircle > Ptr;
    typedef boost::shared_ptr< const OrbitCircle > ConstPtr;

public:
    float radius;
    float step;
    float harc;
    static const float min_arc;
};

class OrbitLine : public GroupOrbit
{
public:
    typedef boost::shared_ptr< OrbitCircle > Ptr;
    typedef boost::shared_ptr< const OrbitCircle > ConstPtr;

public:
    float step;
};

typedef std::map< int, Vector3f > OrbitPosMap;
typedef std::map< int, mpcard > OrbitIndMap;

#endif
