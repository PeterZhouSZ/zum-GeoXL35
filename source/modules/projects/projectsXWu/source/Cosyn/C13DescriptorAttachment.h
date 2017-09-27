//---------------------------------------------------------------------------
#ifndef C13DescriptorAttachmentH
#define C13DescriptorAttachmentH
//---------------------------------------------------------------------------
#include "CommonHdrXWu.h"
//---------------------------------------------------------------------------
#include "Persistent.h"
#include "AttachedData.h"
#include "ObjectListProperty.h"
//---------------------------------------------------------------------------

namespace NAMESPACE_VERSION {

    class C13DescriptorAttachment : public AttachedData {
        DEFINE_CLASS( C13DescriptorAttachment )
    public:
        /// basis vectors in columns
        DMatrixF pcaBasis;
        DVectorF eigValue;
        DMatrixF descriptor;

        const DMatrixF &getPcaBasis() const { return pcaBasis; }
        void setPcaBasis(const DMatrixF &val) { pcaBasis = val; }
        const DVectorF &getEigValue() const { return eigValue; }
        void setEigValue(const DVectorF &val) { eigValue = val; }
        const DMatrixF &getDescriptor() const { return descriptor; }
        void setDescriptor(const DMatrixF &val) { descriptor = val; }

        card32 level;
        float32 scale;
        Vector3f lowerCorner;
        Vector3f upperCorner;
        std::vector< std::vector< card32 > > featureSupports;
        IMPLEMENT_VAR_ARRAY_METHODS_STL(featureSupports);

    public:
        static std::string getDefaultName() { return "DescriptorAtt"; }
        float getDiagonalLength() const;

        virtual void assign(const Object* obj, COPY_CONTEXT *context /* = nullptr */);
        virtual void read(InputObjectStream *s);
        virtual void write(OutputObjectStream *s) const;

    };

}

#endif