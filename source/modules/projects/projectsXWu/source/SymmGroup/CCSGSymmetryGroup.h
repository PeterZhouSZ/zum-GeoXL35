//---------------------------------------------------------------------------
#ifndef CCSG_SYMMETRY_GROUP_H
#define CCSG_SYMMETRY_GROUP_H
//---------------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
//---------------------------------------------------------------------------
#include "PointCloud.h"
#include "ObjectListProperty.h"

//---------------------------------------------------------------------------
#include <RenderingTool.h>
#include <InCorePCTopologyGraph.h>
#include <LocalDescriptor.h>
#include <omp.h>

#include <SymmetryGroup.h>
#include <detector/FeatureLine.h>
#include <SymmetryGroupAttachment.h>

namespace NAMESPACE_VERSION
{
	class DVectorUObjectList;
	class FeatureSet;

	/**
	* Attachment giving information about the symmetry group.
	* This one is often attached to the group list node.
	**/
	class PROJECTSXWU_API CCSGSymmetryGroup : public AttachedData, public RenderableObject
	{
		DEFINE_CLASS(CCSGSymmetryGroup);
	public:
		CCSGSymmetryGroup() : group(nullptr), featureSet(nullptr), id(0), visible(true) {}
		CCSGSymmetryGroup(const std::vector<card32>& symlines, sym::SymmetryGroup* gr)
		{
			featureSet = nullptr;
			group = gr;
			votes = 0;
			lines = symlines;
			visible = true;
			update();
		}
		CCSGSymmetryGroup(const std::vector<card32>& symlines, const std::vector<card32>& coveredlines, sym::SymmetryGroup* gr)
		{
			allLinesCovered = coveredlines;
			featureSet = nullptr;
			visible = true;
			group = gr;
			votes = 0;
			lines = symlines;
			update();
		}
		bool visible;
		virtual void renderGL(SceneObject *obj);
		virtual ~CCSGSymmetryGroup();
		void update() ; 
		std::vector<card32> generatorGIDs;
		IMPLEMENT_VAR_ARRAY_METHODS_STL(generatorGIDs);
		std::vector<card32> lines;
		IMPLEMENT_VAR_ARRAY_METHODS_STL(lines);
		std::vector<mpcard> symmetryPoints;
		IMPLEMENT_VAR_ARRAY_METHODS_STL(symmetryPoints);
		std::vector<card32> allLinesCovered;
		IMPLEMENT_VAR_ARRAY_METHODS_STL(allLinesCovered);
		
		card32 id;

		BoundingBox3f bbox;
		FeatureSet* featureSet;

		//void write(OutputObjectStream *s) const;
		void read(InputObjectStream *s);

		//bool clearGeometrically(UnstructuredInCorePointCloud* baseShape, card32 minpoints, UnstructuredInCorePointCloud** resultShape = nullptr, bool doSymmetricCheck = false, bool doTopologyCheck = true);
		//void mergeLinesFromGroup(CCSGSymmetryGroup* group);
		//void merge(CCSGSymmetryGroup* group);

		std::string key;
		card32 votes;
		sym::SymmetryGroup* group;

		bool operator<(CCSGSymmetryGroup& g) const;

		struct Less
		{
			bool operator() (CCSGSymmetryGroup*& lhs, CCSGSymmetryGroup*& rhs) const { return (*lhs) < (*rhs); }
		};
		struct Greater
		{
			bool operator() (CCSGSymmetryGroup*& lhs, CCSGSymmetryGroup*& rhs) const { return !((*lhs) < (*rhs)); }
		};
		
		bool operator==(const CCSGSymmetryGroup& g) const { return key == g.key; }
		void assign(const Object* obj, COPY_CONTEXT *context);

		static CCSGSymmetryGroup* buildFromSymmetryGroupAttachment(sym::SymmetryGroupAttachment* att);
		static sym::SymmetryGroupAttachment* buildSymmetryGroupAttachment(CCSGSymmetryGroup* att);

		static std::string getDefaultName() { return "CCSGSymmetryGroup"; }
	};

} // end namespace

#endif
