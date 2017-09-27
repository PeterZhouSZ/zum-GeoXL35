#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "Util/SceneAndPointCloudTools.h"
//---------------------------------------------------------------------------
#include "SceneGraphTools.h"
//---------------------------------------------------------------------------

namespace {
    int DR_FRAME = 0;
}

SGListNode* NU::GetOrCreateSGListNode(Scene* scene, const std::string& fullPath, bool animNode)
{
    // try to find existing node
    SceneGraphNode * node = getSceneGraphNode(scene, fullPath);
    SGListNode * ret = dynamic_cast<SGListNode*>(node);
    if (node && !ret)
        error(std::string("GetOrCreateSGListNode() - Node already exists, but not a list node! Path: ") + fullPath + "\n");

    if (ret)
        return ret;

    // node does not exists, -> create
    std::string path = extractSGPath(fullPath);
    //	if( path.length() > 0 && path[path.length)-1]=='/')
    //		path = path.substr(0,path.length()-1);

    SceneGraphNode* pathNode = getSceneGraphNode(scene, path);
    if (pathNode == nullptr)
        error(std::string("GetOrCreateSGListNode() - Path does not exists, tried to create list node: ") + fullPath + "\n");

    if (!animNode)
        ret = new SGListNode;
    else
        ret = new SGRelativeTimeAnimationNode;
    ret->setName(extractSGNode(fullPath));
    pathNode->addChildNode(ret);
    return ret;
}

void NU::TransformPointCloud(UnstructuredInCorePointCloud* source, const Matrix4f& trans)
{
    PointSet *tmpPS = source->getPointSet();
    AAT posAAT = tmpPS->getAAT("position");
    BasicPointCloudIterator *sourceIt = dynamic_cast<BasicPointCloudIterator*>(source->createIterator(SOIT::CAP_BASIC_PC));
    ModifyablePointCloudIterator *sourceModIt = dynamic_cast<ModifyablePointCloudIterator*>(sourceIt);
    sourceIt->reset();
    while (!sourceIt->atEnd()) {
        Vector3f p = sourceIt->get3f(posAAT);
        Vector3f p_t = transformVector3f(trans, p);
        sourceModIt->set3f(posAAT, p_t);
        sourceIt->next();
    }
}
