#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "RepBoxLoader.h"
#include "Util/ColorSchemer.hpp"
#include "Util\ExtraGL.hpp"
//---------------------------------------------------------------------------
#include "InterfaceGlobals.h"
#include "FileDialogs.h"
#include <fstream>
#include <boost/algorithm/string/trim.hpp>
#include <boost/algorithm/string/split.hpp>
//---------------------------------------------------------------------------

namespace {
    int DR_FRAME = 0;
}

namespace {
    float BOX_LEN_MAX = 0.2f;
}

RepBoxLoader::RepBoxLoader(void) : data_(nullptr)
{
}

void RepBoxLoader::SetPC(UICPC* pc)
{
    data_ = pc;
}

std::deque<RepBox::Ptr> RepBoxLoader::GetClassList(const unsigned& num)
{
    return boxRecord[num];
}

void RepBoxLoader::GetTemplateList(std::deque<RepBox::Ptr>* pTempList)
{
    std::deque<RepBox::Ptr>& tempList = *pTempList;
    for (unsigned ci = 0; ci < boxRecord.size(); ++ci) {
        tempList.push_back(boxRecord[ci][0]);
    }
}

void RepBoxLoader::GetRepBoxList(std::deque<RepBox::Ptr>* pBoxList)
{
    std::deque<RepBox::Ptr>& boxList = *pBoxList;
    for (unsigned ci = 0; ci < boxRecord.size(); ++ci) {
        std::deque<RepBox::Ptr>& list = boxRecord[ci];
        for (unsigned ei = 0; ei < list.size(); ++ei) {
            boxList.push_back(list[ei]);
        }
    }
}

void RepBoxLoader::DrawBoxes(
    const std::deque<RepBox::Ptr>& boxList,
    const Vector3f& color, const float32& linewidth
    )
{
    for (mpcard i_box = 0; i_box < boxList.size(); i_box++) {
        float32 lw = (i_box == 0) ? linewidth * 4 : linewidth;
        RepBoxLoader::DrawBox(
            boxList[i_box],
            color, lw);
    }
}

void RepBoxLoader::DrawBoxes(const std::deque< std::deque<RepBox::Ptr> >& boxList,
    const float32& linewidth)
{
    debugRenderer->beginRenderJob_OneFrame("load_box_", DR_FRAME++, true);
    for (mpcard i_class = 0; i_class < boxList.size(); i_class++) {
        DrawBoxes(boxList[i_class],
            ColorSchemer::GetJetColor(i_class, boxList.size()), linewidth);
    }
    debugRenderer->endRenderJob();
}

void RepBoxLoader::DrawBoxes(const float32& linewidth)
{
    RepBoxLoader::DrawBoxes(boxRecord, linewidth);
}

void RepBoxLoader::LoadSymmetries(void)
{
    std::string filename = FileDialogs::getOpenFileName(InterfaceGlobals::getMainWindow(), "load GT",
        "*.box");
    std::ifstream fs;
    std::string line;
    fs.open(filename.c_str(), std::ios::binary);
    if (!fs.is_open() || fs.fail()) {
        error("Failed to open file...");
        return;
    }

    // # $number$ matrices
    // # @@
    // x x x x
    // x x x x
    // x x x x
    // x x x x
    // # @@
    // ...
    getline(fs, line);
    std::vector<std::string> st;
    while (getline(fs, line))
    {
        if ('#' != line.at(0)) break;
        Eigen::Matrix4f mat;
        for (unsigned rr = 0; rr < 4; ++rr) {
            getline(fs, line);
            boost::trim(line);
            boost::split(st, line, boost::is_any_of("\t\r "), boost::token_compress_on);
            if (4 != st.size()) {
                error("wrong matrix format");
                error(line);
                return;
            }
            for (unsigned cc = 0; cc < 4; ++cc) {
                float value;
                value = atof(st.at(cc).c_str());
                mat(rr, cc) = value;
            }
        }
        symmetries_.push_back(mat);
        if (0) {
            std::ostringstream ss;
            ss << mat;
            debugOutput << ss.str() << "\n";
        }
    }
    debugOutput << "loaded " << symmetries_.size() << " symmetries.\n";
}

void RepBoxLoader::LoadGT(void)
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

    boxRecord.clear();
    BOX_LEN_MAX = -1;

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

            for (unsigned d = 0; d < 3; ++d) {
                if (BOX_LEN_MAX < boxRecord[i_class][i_box]->coeffs[d])
                    BOX_LEN_MAX = boxRecord[i_class][i_box]->coeffs[d];
            }
        }
    }
    fs.close();
    
    std::deque< std::deque<RepBox::Ptr> >::iterator it;
    for (it = boxRecord.begin(); it != boxRecord.end();) {
        if (it->empty()) it = boxRecord.erase(it);
        else ++it;
    }

    //// render the record
    //debugRenderer->beginRenderJob_OneFrame("load_box_", DR_FRAME++, true);
    //for (mpcard i_class = 0; i_class < boxRecord.size(); i_class++)
    //{
    //    for (mpcard i_box = 0; i_box < boxRecord[i_class].size(); i_box++)
    //    {
    //        float32 linewidth;
    //        if (i_box == 0)
    //        {
    //            linewidth = 8;
    //        }
    //        else
    //        {
    //            linewidth = 2;
    //        }
    //        DrawBox(boxRecord[i_class][i_box], color_list[i_class], linewidth);
    //    }
    //}
    //debugRenderer->endRenderJob();

    debugOutput << "repetition loaded as boxes\n";
}

void RepBoxLoader::SaveGT(void){
    std::string filename = FileDialogs::getSaveFileName(InterfaceGlobals::getMainWindow(), "save GT", "*.box");
    if (filename != "")
    {
        std::ofstream fs;
        std::string line;
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

void RepBoxLoader::DrawBox(RepBox::Ptr box, const Vector3f& color, const float32& linewidth)
{
    BoundingBox3f bbox(NULL_VECTOR3F);
    int valueSet[2] = { -1, 1 };
    for (unsigned x = 0; x < 2; ++x) {
        for (unsigned y = 0; y < 2; ++y) {
            for (unsigned z = 0; z < 2; ++z) {
                Vector3f axis = makeVector3f(valueSet[x], valueSet[y], valueSet[z]);
                for (unsigned d = 0; d < 3; ++d) {
                    axis[d] *= box->coeffs[d];
                }
                bbox.addPoint(axis);
            }
        }
    }
    Matrix4f T = expand3To4(box->basis);
    T[3] = expand3To4(box->centroid);
    debugRenderer->addBoundingBox(
        T,
        bbox,
        color, linewidth
        );

    debugRenderer->addLine(box->centroid, box->centroid + box->basis[0] * abs(box->coeffs[0]), makeVector4f(1, 1, 1, 1), makeVector4f(1, 0, 0, 1), linewidth, false, false);
    debugRenderer->addLine(box->centroid, box->centroid + box->basis[1] * abs(box->coeffs[1]), makeVector4f(1, 1, 1, 1), makeVector4f(0, 1, 0, 1), linewidth, false, false);
    debugRenderer->addLine(box->centroid, box->centroid + box->basis[2] * abs(box->coeffs[2]), makeVector4f(1, 1, 1, 1), makeVector4f(0, 0, 1, 1), linewidth, false, false);
    //debugRenderer->addFastSphere(box->cenref, BOX_LEN_MAX * .2f, makeVector3f(1, 0, 1), false);
    debugRenderer->addFastSphere(box->centroid, linewidth * .04f, color, false);
}

Vector3f RepBoxLoader::MapGroupColor(const unsigned& groupID)
{
    return ColorSchemer::GetJetColor(groupID, boxRecord.size());
}

void RepBoxLoader::glDrawBox(RepBox::Ptr box, const Vector3f& color, const float32& linewidth)
{
    //   2 - 6
    //  /   /|
    // 3 - 7 4
    // |   |/
    // 1 - 5
    //
    // y
    // |
    // o --> x

    std::deque<Vector3f> vertices;
    int valueSet[2] = { -1, 1 };
    for (unsigned x = 0; x < 2; ++x) {
        for (unsigned y = 0; y < 2; ++y) {
            for (unsigned z = 0; z < 2; ++z) {
                Vector3f axis = makeVector3f(valueSet[x], valueSet[y], valueSet[z]);
                for (unsigned d = 0; d < 3; ++d) {
                    axis[d] *= box->coeffs[d];
                }
                vertices.push_back(axis);
            }
        }
    }
    Matrix4f T = expand3To4(box->basis);
    T[3] = expand3To4(box->centroid);
    for (unsigned ii = 0; ii < 8; ++ii) {
        vertices[ii] = transformVector3f(T, vertices[ii]);
    }

    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glColor3fv(color.data());
    glLineWidth(linewidth);
    glBegin(GL_QUADS);
    // left
    glVertex3fv(vertices[0].data());
    glVertex3fv(vertices[1].data());
    glVertex3fv(vertices[3].data());
    glVertex3fv(vertices[2].data());
    // right
    glVertex3fv(vertices[4].data());
    glVertex3fv(vertices[5].data());
    glVertex3fv(vertices[7].data());
    glVertex3fv(vertices[6].data());
    // top
    glVertex3fv(vertices[2].data());
    glVertex3fv(vertices[6].data());
    glVertex3fv(vertices[7].data());
    glVertex3fv(vertices[3].data());
    // bottom
    glVertex3fv(vertices[0].data());
    glVertex3fv(vertices[4].data());
    glVertex3fv(vertices[5].data());
    glVertex3fv(vertices[1].data());
    // front
    glVertex3fv(vertices[1].data());
    glVertex3fv(vertices[5].data());
    glVertex3fv(vertices[7].data());
    glVertex3fv(vertices[3].data());
    // back
    glVertex3fv(vertices[0].data());
    glVertex3fv(vertices[4].data());
    glVertex3fv(vertices[6].data());
    glVertex3fv(vertices[2].data());
    glEnd();
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    SolidSphere sphere(linewidth * .04);
    sphere.draw(box->centroid[0], box->centroid[1], box->centroid[2]);
}
