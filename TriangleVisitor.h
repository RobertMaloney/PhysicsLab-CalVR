#ifndef _TRIANGLEVISITOR_H
#define _TRIANGLEVISITOR_H

#include <algorithm>

#include <osg/TriangleFunctor>
#include <osg/Geode>
#include <osg/NodeVisitor>
#include <osg/Vec3>
#include <osg/Matrix>

struct Triangle
{
    osg::Vec3f v1;
    osg::Vec3f v2;
    osg::Vec3f v3;
};


class TriangleVisitor : public osg::NodeVisitor
{
    static std::vector< Triangle > * _triangles;
    static osg::Matrixd _matrix;
    static osg::Vec3 max, min;
    protected:
    
        struct WorldTriangleAdd
        {
            void operator() (const osg::Vec3& v1,const osg::Vec3& v2,const osg::Vec3& v3, bool) const
            {
                Triangle t;
                t.v1 = v1 * _matrix;
                t.v2 = v2 * _matrix;
                t.v3 = v3 * _matrix;
                _triangles->push_back(t);
                
                //min.x() = min( min( min.x(), v1.x() ), min( v2.x(), v3.x() ) );
                //min.y() = min( min( min.y(), v1.y() ), min( v2.y(), v3.y() ) );
                //min.z() = min( min( min.z(), v1.z() ), min( v2.z(), v3.z() ) );
            }
        };

        // functor for adding triangles
        osg::TriangleFunctor<WorldTriangleAdd> tf;


    public:
        TriangleVisitor();
        ~TriangleVisitor();
        
        virtual void apply(osg::Geode& geode);

        std::vector< Triangle > * getTriangles() { return _triangles; };
        
        void resetTriangles() { _triangles->clear(); };
};

#endif
