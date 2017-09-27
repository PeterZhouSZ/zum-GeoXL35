#ifndef SceneAndPointCloudTools_H
#define SceneAndPointCloudTools_H
//---------------------------------------------------------------------------
#include "CommonHdrXWu.h"
//---------------------------------------------------------------------------
#include "SGListNode.h"
#include "SGRelativeTimeAnimationNode.h"
//---------------------------------------------------------------------------

namespace NU {

    SGListNode* GetOrCreateSGListNode(Scene* scene, const std::string& fullPath, bool animNode);

    void TransformPointCloud(UnstructuredInCorePointCloud* source, const Matrix4f& trans);

}

#endif
