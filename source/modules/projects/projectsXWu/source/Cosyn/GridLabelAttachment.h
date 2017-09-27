//---------------------------------------------------------------------------
#ifndef GridLabelAttachmentH
#define GridLabelAttachmentH
//---------------------------------------------------------------------------
#include "CommonHdrXWu.h"
//---------------------------------------------------------------------------
#include "Persistent.h"
#include "AttachedData.h"
#include "ObjectListProperty.h"
//---------------------------------------------------------------------------

namespace NAMESPACE_VERSION {

    class GridLabelAttachment : public AttachedData {
        DEFINE_CLASS( GridLabelAttachment )
    public:
        std::vector<Vector3f> cell_cen;
        IMPLEMENT_VAR_ARRAY_METHODS_STL(cell_cen);
        std::vector<unsigned> labels;
        IMPLEMENT_VAR_ARRAY_METHODS_STL(labels);
        float cell_size;

    public:
        static std::string getDefaultName() { return "GridLabelAtt"; }

        void VisualizeGrid(void);

        virtual void assign(const Object* obj, COPY_CONTEXT *context /* = nullptr */);
        virtual void read(InputObjectStream *s);
        virtual void write(OutputObjectStream *s) const;
    };

}

#endif