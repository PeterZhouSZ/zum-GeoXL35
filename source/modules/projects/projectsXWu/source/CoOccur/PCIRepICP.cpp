#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "PCIRepICP.h"
#include "InterfaceGlobals.h"
#include "FileDialogs.h"
#include "SceneEditorWidget.h"
#include "LCHierarchyRenderer.h"
#include "GLShaderMaterial.h"
#include "SGlistNode.h"
//---------------------------------------------------------------------------
#include "Util/ColorSchemer.hpp"
#include "Util\DampingICP.h"
#include <fstream>
#include <boost/algorithm/string/trim.hpp>
#include <boost/algorithm/string/split.hpp>
//---------------------------------------------------------------------------
namespace {
    int DR_FRAME = 0;
}

//---------------------------------------------------------------------------
IMPLEMENT_CLASS(PCIRepICP, 0)
{
    BEGIN_CLASS_INIT(PCIRepICP);
    ADD_NOARGS_METHOD(loadGT);
    ADD_NOARGS_METHOD(saveGT);
    ADD_NOARGS_METHOD(GeoAlign);
}
//---------------------------------------------------------------------------

PCIRepICP::PCIRepICP(void)
{
}

PCIRepICP::~PCIRepICP(void)
{
}

void PCIRepICP::loadGT(void)
{
    std::string filename = FileDialogs::getOpenFileName(InterfaceGlobals::getMainWindow(), "load GT",
        "*.box");
    std::ifstream fs;
    std::string line;
    fs.open(filename.c_str(), std::ios::binary);
    if (!fs.is_open() || fs.fail()){
        error("Failed to open file...");
        return;
    }

    std::vector<std::string> st;
    // skip the first two lines
    getline(fs, line);
    getline(fs, line);

    // read data
    getline(fs, line);
    boost::trim(line);
    boost::split(st, line, boost::is_any_of("\t\r "), boost::token_compress_on);
    float32 value;
    value = atof(st.at(0).c_str());
    boxRecord.resize((int32)value);
    for (mpcard i_class = 0; i_class < boxRecord.size(); i_class++)
    {
        getline(fs, line);
        boost::trim(line);
        boost::split(st, line, boost::is_any_of("\t\r "), boost::token_compress_on);
        value = atof(st.at(0).c_str());
        boxRecord[i_class].resize((int32)value);
    }
    for (mpcard i_class = 0; i_class < boxRecord.size(); i_class++)
    {
        for (mpcard i_box = 0; i_box < boxRecord[i_class].size(); i_box++)
        {
            getline(fs, line);
            boost::trim(line);
            boost::split(st, line, boost::is_any_of("\t\r "), boost::token_compress_on);
            boxRecord[i_class][i_box] = boost::shared_ptr<RepBox>(new RepBox);
            boxRecord[i_class][i_box]->basis[0][0] = atof(st.at(0).c_str());
            boxRecord[i_class][i_box]->basis[0][1] = atof(st.at(1).c_str());
            boxRecord[i_class][i_box]->basis[0][2] = atof(st.at(2).c_str());
            boxRecord[i_class][i_box]->basis[1][0] = atof(st.at(3).c_str());
            boxRecord[i_class][i_box]->basis[1][1] = atof(st.at(4).c_str());
            boxRecord[i_class][i_box]->basis[1][2] = atof(st.at(5).c_str());
            boxRecord[i_class][i_box]->basis[2][0] = atof(st.at(6).c_str());
            boxRecord[i_class][i_box]->basis[2][1] = atof(st.at(7).c_str());
            boxRecord[i_class][i_box]->basis[2][2] = atof(st.at(8).c_str());
            boxRecord[i_class][i_box]->coeffs[0] = atof(st.at(9).c_str());
            boxRecord[i_class][i_box]->coeffs[1] = atof(st.at(10).c_str());
            boxRecord[i_class][i_box]->coeffs[2] = atof(st.at(11).c_str());
            boxRecord[i_class][i_box]->centroid[0] = atof(st.at(12).c_str());
            boxRecord[i_class][i_box]->centroid[1] = atof(st.at(13).c_str());
            boxRecord[i_class][i_box]->centroid[2] = atof(st.at(14).c_str());
            boxRecord[i_class][i_box]->cenref[0] = atof(st.at(15).c_str());
            boxRecord[i_class][i_box]->cenref[1] = atof(st.at(16).c_str());
            boxRecord[i_class][i_box]->cenref[2] = atof(st.at(17).c_str());
            boxRecord[i_class][i_box]->radiusref = atof(st.at(18).c_str());
            boxRecord[i_class][i_box]->label = atof(st.at(19).c_str());
        }
    }
    fs.close();

    // render the record
    debugRenderer->beginRenderJob_OneFrame("load_box_", DR_FRAME++, true);
    for (mpcard i_class = 0; i_class < boxRecord.size(); i_class++)
    {
        for (mpcard i_box = 0; i_box < boxRecord[i_class].size(); i_box++)
        {
            float32 linewidth;
            if (i_box == 0)
            {
                linewidth = 8;
            }
            else
            {
                linewidth = 2;
            }
            drawBox(boxRecord[i_class][i_box], color_list[i_class], linewidth);
        }
    }
    debugRenderer->endRenderJob();
}

void PCIRepICP::saveGT(void){
    std::string filename = FileDialogs::getSaveFileName(InterfaceGlobals::getMainWindow(), "save GT", "*.box");
    if (filename != "")
    {
        std::ofstream fs;
        fs.open(filename.c_str(), std::ios::binary);
        fs << "# .gt v0.0 - 3D Repetition Benchmark box format\n";
        fs << "# basis[0][0] basis[0][1] basis[0][2] basis[1][0] basis[1][1] basis[1][2] basis[2][0] basis[2][1] basis[2][2] coeffs[0] coeffs[1] coeffs[2] centroid[0] centroid[1] centroid[2] cenref[0] cenref[1] cenref[2] radiusref label" << "\n";
        fs << boxRecord.size() << "\n";
        for (mpcard i_class = 0; i_class < boxRecord.size(); i_class++) {
            fs << boxRecord[i_class].size() << "\n";
        }
        for (mpcard i_class = 0; i_class < boxRecord.size(); i_class++)
        {
            for (mpcard i_box = 0; i_box < boxRecord[i_class].size(); i_box++)
            {
                fs << boxRecord[i_class][i_box]->basis[0][0] << " " << boxRecord[i_class][i_box]->basis[0][1] << " " << boxRecord[i_class][i_box]->basis[0][2] << " ";
                fs << boxRecord[i_class][i_box]->basis[1][0] << " " << boxRecord[i_class][i_box]->basis[1][1] << " " << boxRecord[i_class][i_box]->basis[1][2] << " ";
                fs << boxRecord[i_class][i_box]->basis[2][0] << " " << boxRecord[i_class][i_box]->basis[2][1] << " " << boxRecord[i_class][i_box]->basis[2][2] << " ";
                fs << boxRecord[i_class][i_box]->coeffs[0] << " " << boxRecord[i_class][i_box]->coeffs[1] << " " << boxRecord[i_class][i_box]->coeffs[2] << " ";
                fs << boxRecord[i_class][i_box]->centroid[0] << " " << boxRecord[i_class][i_box]->centroid[1] << " " << boxRecord[i_class][i_box]->centroid[2] << " ";
                fs << boxRecord[i_class][i_box]->cenref[0] << " " << boxRecord[i_class][i_box]->cenref[1] << " " << boxRecord[i_class][i_box]->cenref[2] << " ";
                fs << boxRecord[i_class][i_box]->radiusref << " ";
                fs << boxRecord[i_class][i_box]->label;
                fs << "\n";
            }
        }
        fs.close();
    }
}

void PCIRepICP::drawQuad(
    const Vector3f &p1, const Vector3f &p2, const Vector3f &p3, const Vector3f &p4,
    const Vector3f &color, const float &linewidth)
{
    debugRenderer->addLine(p1, p2, color, color, linewidth, false, 0);
    debugRenderer->addLine(p2, p3, color, color, linewidth, false, 0);
    debugRenderer->addLine(p3, p4, color, color, linewidth, false, 0);
    debugRenderer->addLine(p4, p1, color, color, linewidth, false, 0);
}

void PCIRepICP::drawCubeTR(
    const Vector3f &pbegin, const Vector3f &pend,
    const Vector3f &color, const float &linewidth,
    Vector3f T, Matrix3f R)
{
    Vector3f p1, p2, p3, p4;
    // face one 
    p1 = pbegin;
    p2 = makeVector3f(pbegin[0], pbegin[1], pend[2]);
    p3 = makeVector3f(pend[0], pbegin[1], pend[2]);
    p4 = makeVector3f(pend[0], pbegin[1], pbegin[2]);
    p1 = R * p1 + T;
    p2 = R * p2 + T;
    p3 = R * p3 + T;
    p4 = R * p4 + T;
    drawQuad(p1, p2, p3, p4, color, linewidth);

    // face two
    p1 = makeVector3f(pbegin[0], pbegin[1], pend[2]);
    p2 = makeVector3f(pend[0], pbegin[1], pend[2]);
    p3 = makeVector3f(pend[0], pend[1], pend[2]);
    p4 = makeVector3f(pbegin[0], pend[1], pend[2]);
    p1 = R * p1 + T;
    p2 = R * p2 + T;
    p3 = R * p3 + T;
    p4 = R * p4 + T;
    drawQuad(p1, p2, p3, p4, color, linewidth);

    // face three
    p1 = makeVector3f(pbegin[0], pend[1], pbegin[2]);
    p2 = makeVector3f(pbegin[0], pend[1], pend[2]);
    p3 = makeVector3f(pend[0], pend[1], pend[2]);
    p4 = makeVector3f(pend[0], pend[1], pbegin[2]);
    p1 = R * p1 + T;
    p2 = R * p2 + T;
    p3 = R * p3 + T;
    p4 = R * p4 + T;
    drawQuad(p1, p2, p3, p4, color, linewidth);

    // face four
    p1 = makeVector3f(pbegin[0], pbegin[1], pbegin[2]);
    p2 = makeVector3f(pend[0], pbegin[1], pbegin[2]);
    p3 = makeVector3f(pend[0], pend[1], pbegin[2]);
    p4 = makeVector3f(pbegin[0], pend[1], pbegin[2]);
    p1 = R * p1 + T;
    p2 = R * p2 + T;
    p3 = R * p3 + T;
    p4 = R * p4 + T;
    drawQuad(p1, p2, p3, p4, color, linewidth);

    // face five
    p1 = pbegin;
    p2 = makeVector3f(pbegin[0], pbegin[1], pend[2]);
    p3 = makeVector3f(pbegin[0], pend[1], pend[2]);
    p4 = makeVector3f(pbegin[0], pend[1], pbegin[2]);
    p1 = R * p1 + T;
    p2 = R * p2 + T;
    p3 = R * p3 + T;
    p4 = R * p4 + T;
    drawQuad(p1, p2, p3, p4, color, linewidth);

    // face six
    p1 = makeVector3f(pend[0], pbegin[1], pbegin[2]);
    p2 = makeVector3f(pend[0], pbegin[1], pend[2]);
    p3 = makeVector3f(pend[0], pend[1], pend[2]);
    p4 = makeVector3f(pend[0], pend[1], pbegin[2]);
    p1 = R * p1 + T;
    p2 = R * p2 + T;
    p3 = R * p3 + T;
    p4 = R * p4 + T;
    drawQuad(p1, p2, p3, p4, color, linewidth);
}

void PCIRepICP::drawBox(RepBox::Ptr box, Vector3f color, float32 lindwidth)
{
    drawCubeTR(box->coeffs, -box->coeffs, color, lindwidth, box->centroid, box->basis);
    debugRenderer->addLine(box->centroid, box->centroid + box->basis[0] * abs(box->coeffs[0]), makeVector4f(1, 1, 1, 1), makeVector4f(1, 0, 0, 1), lindwidth, false, false);
    debugRenderer->addLine(box->centroid, box->centroid + box->basis[1] * abs(box->coeffs[1]), makeVector4f(1, 1, 1, 1), makeVector4f(0, 1, 0, 1), lindwidth, false, false);
    debugRenderer->addLine(box->centroid, box->centroid + box->basis[2] * abs(box->coeffs[2]), makeVector4f(1, 1, 1, 1), makeVector4f(0, 0, 1, 1), lindwidth, false, false);
    debugRenderer->addFastSphere(box->cenref, 0.25f, makeVector3f(1, 0, 1), false);
}

// UI inherited rendering functions
void PCIRepICP::mouseDown(int32 x, int32 y, MouseButtons buttonsState, Modifiers modifiersState)
{
    mouseDownPassToCam(x, y, buttonsState, modifiersState);
}

void PCIRepICP::mouseMoved(int32 x, int32 y)
{
    mouseMovedPassToCam(x, y);
}

void PCIRepICP::mouseUp(int32 x, int32 y, MouseButtons buttonsState, Modifiers modifiersState)
{
    mouseUpPassToCam(x, y, buttonsState, modifiersState);
}

void PCIRepICP::mouseWheelRotated(int32 rotatedDelta, Modifiers modifiersState)
{
    mouseWheelRotatedPassToCam(rotatedDelta, modifiersState);
}

void PCIRepICP::areaResize(card32 width, card32 height)
{
    areaResizePassToCam(width, height);
}

void PCIRepICP::keyDown(GeneralKey key)
{
    keyDownPassToCam(key);
}

void PCIRepICP::keyUp(GeneralKey key)
{
    keyUpPassToCam(key);
}

void PCIRepICP::glDrawTool(GLContext *glContext) {}

void PCIRepICP::connectToSceneImpl(Scene * scene, OnSceneChangeCallback *onChange, QWidget *sceneEditorWidget, QWidget* toolWidget)
{
    SceneEditorWidget* sceneEditorW = dynamic_cast<NAMESPACE_VERSION::SceneEditorWidget*>(sceneEditorWidget);
    sceneEditorW->on_action_Auto_Layout_triggered(true);
    scene->getRootState()->staticState->backgroundColor = makeVector4f(1.0f, 1.0f, 1.0f, 1.0f); // white
    //scene->getRootState()->staticState->backgroundColor = makeVector4f(0.6f, 0.6f, 0.9f, 1.0f); // bright
    scene->getRootState()->staticState->rfShowLocalCoords = false;
    LCHierarchyRenderer* lchRenderer = dynamic_cast<NAMESPACE_VERSION::LCHierarchyRenderer*>(sceneEditorW->getCurrentRenderer());
    if (lchRenderer)
    {
        lchRenderer->getState()->getTraversalState()->PARAM_drawCloudSelection = false;
    }
    for (unsigned int n = 0; n < scene->getNumGLMaterials(); n++){
        GLShaderMaterial* mat = dynamic_cast<GLShaderMaterial*>(scene->getGLMaterial(n));
        if (mat != NULL){
            mat->setForceTriangleNormal(true);
        }
    }
}

void PCIRepICP::GeoAlign(void)
{
    if (boxRecord.empty()) {
        warning("PCIRepICP::GeoAlign() - box record not found.");
        return;
    }
    UnstructuredInCorePointCloud* pcFullres = dynamic_cast<UnstructuredInCorePointCloud*>(
        getPointCloud(getScene(), "root/pcFullres"));
    if (nullptr == pcFullres) {
        warning("PCIRepICP::GeoAlign() - no fullres point cloud.");
        return;
    }

    debugOutput << "\n";
    debugOutput << "########################################\n";
    debugOutput << "# geometric validation (icp) \n";
    debugOutput << "########################################\n";
    debugOutput << "\n";
    X4_TIMER_START(geometric_validation);

    const float median_dist = getMedianPointDistance(pcFullres);
    PointSet* psFullres = pcFullres->getPointSet();
    float bb_diag_length;
    {
        const unsigned numPoints = psFullres->getNumEntries();
        const AAT POSITION = psFullres->getAAT("position");
        BoundingBox3f bb(psFullres->get3f(0, POSITION));
        for (unsigned ii = 1; ii < numPoints; ++ii)
            bb.addPoint(psFullres->get3f(ii, POSITION));
        bb_diag_length = bb.getDiagonalLength();
    }
    const float min_residual = median_dist * 0.5f;
    FastSphereQuerry* queryBox = new FastSphereQuerry(pcFullres);
    DampingICP* icp = new DampingICP(pcFullres);
    icp->setupParameters(
        10,                                 // number of iterations
        median_dist,                        // median point distance
        10.f,                               // outlier distance factor
        0.8f,                               // minimum inlier percentage
        bb_diag_length * 0.05f,             // allowed move distance
        min_residual,                       // maximal residual tolerance
        min_residual * 0.1f,                // convergence difference threshold
        false                               // show_icl_icp
        );

    std::deque<RepBox::Ptr>& boxRecs = boxRecord[0];
    const unsigned numBox = boxRecs.size();
    enum EMarkT { EMarkTUnchecked = 0, EMarkTChecked = 1 };
    std::deque<int> boxMark(numBox, EMarkTUnchecked);
    debugOutput << str(boost::format("total %1% boxes\n")
        % numBox
        );
    for (unsigned bi = 0; bi < numBox; ++bi) {
        if (EMarkTChecked == boxMark[bi]) continue;
        debugOutput << str(boost::format("base box: %1%\n")
            % bi
            );
        const RepBox::Ptr& boxBase = boxRecs[bi];
        boxMark[bi] = EMarkTChecked;

        Matrix3f frameBasis;
        float radiusBox;
        {
            std::array<float, 3> radiusBasis;
            for (unsigned ii = 0; ii < 3; ++ii) {
                radiusBasis[ii] = norm(boxBase->basis[ii]);
                frameBasis[ii] = normalize(boxBase->basis[ii]);
            }
            radiusBox = *std::max_element(radiusBasis.begin(), radiusBasis.end());
            radiusBox *= 2.f; // expand radius
        }
        const Vector3f& cenBase = boxBase->centroid;
        mpcard numAllPoints; mpcard* allIndices;
        queryBox->querry(cenBase, radiusBox, &allIndices, numAllPoints);
        std::deque<mpcard> queryIndice(allIndices, allIndices + numAllPoints);
        PointSet* boxBasePS = psFullres->subset(queryIndice);

        for (unsigned bj = bi + 1; bj < numBox; ++bj) {
            if (EMarkTChecked == boxMark[bj]) continue;
            debugOutput << str(boost::format("  aligning: %1% --> %2% ...")
                % bi % bj
                );

            RepBox::Ptr& boxRef = boxRecs[bj];
            Matrix3f frameRef;
            for (unsigned ii = 0; ii < 3; ++ii) {
                frameRef[ii] = normalize(boxRef->basis[ii]);
            }

            Matrix4f frameBasis4 = expand3To4(frameBasis);
            frameBasis4[3] = expand3To4(cenBase);
            Matrix4f frameRef4 = expand3To4(frameRef);
            frameRef4[3] = expand3To4(boxRef->centroid);
            Matrix4f Ticp = frameRef4 * invertFrame(frameBasis4);
            float icp_residual = icp->calculateResidualAfterICP(boxBasePS, Ticp);
            if (1e10f < icp_residual) {
                debugOutput << str(boost::format(" not aligned\n")
                    );
                continue; // not aligned
            }

            Matrix4f Trec = invertMatrix(Ticp);
            Trec[3] = Trec[3] + expand3To4(boxRef->centroid - cenBase);
            Matrix4f Trot = expand3To4(frameRef * invertFrame(frameBasis)) * expand3To4(shrink4To3(Trec));

            const bool show_icp = false;
            if (show_icp)
            {
                debugRenderer->beginRenderJob_OneFrame("icp_align_", DR_FRAME++);
                debugRenderer->setSmoothLine(true);
                Vector3f sourceColor = makeVector3f(1.f, 0.f, 0.f);
                Vector3f destColor = makeVector3f(0.f, 0.f, 1.f);
                Vector3f diffColor = makeVector3f(0.f, 1.f, 0.f);
                Vector3f subsetColor1 = makeVector3f(0.8f, 0.8f, 0.f);
                Vector3f subsetColor2 = makeVector3f(0.f, 0.8f, 0.8f);

                debugRenderer->addPoint(cenBase,
                    sourceColor);
                debugRenderer->addPoint(boxRef->centroid,
                    destColor);
                debugRenderer->addLine(
                    cenBase, boxRef->centroid,
                    sourceColor, destColor, 10.0f);
                debugRenderer->addLine(
                    transformVector3f(Trec, boxRef->centroid), boxRef->centroid,
                    diffColor, diffColor, 10.0f);

                AAT positionAAT = boxBasePS->getAAT("position");
                for (unsigned ii = 0; ii < queryIndice.size(); ++ii) {
                    const Vector3f& pos = boxBasePS->get3f(ii, positionAAT);
                    debugRenderer->addPoint(pos,
                        subsetColor1);
                    const Vector3f& pos_t = transformVector3f(Ticp, pos);
                    debugRenderer->addPoint(pos_t,
                        subsetColor2);
                    //debugRenderer->addLine(
                    //    pos, pos_t,
                    //    subsetColor1, subsetColor2, 10.0f);
                }

                for (unsigned ii = 0; ii < 3; ++ii) {
                    Vector3f vax = transformVector3f(Trot, frameRef[ii])
                        * norm(boxRef->basis[ii]);
                    debugRenderer->addLine(
                        boxRef->centroid, boxRef->centroid + vax,
                        makeVector3f(1.f, 0.f, 0.f), makeVector3f(1.f, 0.f, 0.f),
                        3);
                    debugRenderer->addLine(
                        boxRef->centroid, boxRef->centroid + boxRef->basis[ii],
                        makeVector3f(0.f, 0.f, 1.f), makeVector3f(0.f, 0.f, 1.f),
                        3);
                }

                debugRenderer->endRenderJob();
            }

            {
                for (unsigned ii = 0; ii < 3; ++ii) {
                    boxRef->basis[ii] = transformVector3f(Trot, frameRef[ii])
                        * norm(boxRef->basis[ii]);
                }
                boxRef->centroid = transformVector3f(Trec, boxRef->centroid);
                boxRef->cenref = transformVector3f(Trec, boxRef->cenref);
            }
            boxMark[bj] = EMarkTChecked;
            debugOutput << str(boost::format(" aligned, and transformed\n")
                );

            //break;
        }

        delete boxBasePS;

        //break;
    }

    delete icp;
    delete queryBox;

    debugRenderer->beginRenderJob_OneFrame("load_box_", DR_FRAME++, true);
    for (mpcard i_class = 0; i_class < boxRecord.size(); i_class++)
    {
        for (mpcard i_box = 0; i_box < boxRecord[i_class].size(); i_box++)
        {
            float32 linewidth;
            if (i_box == 0)
            {
                linewidth = 8;
            }
            else
            {
                linewidth = 2;
            }
            drawBox(boxRecord[i_class][i_box], color_list[i_class], linewidth);
        }
    }
    debugRenderer->endRenderJob();

    X4_TIMER_STOP(geometric_validation);
    debugOutput << "\n";
    debugOutput << "########################################\n";
    debugOutput << "# Done \n";
    debugOutput << "########################################\n";
    debugOutput << "\n";
}
