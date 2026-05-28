/**
* This file is part of SUPER
*
* Copyright 2025 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/SUPER>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* SUPER is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* SUPER is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with SUPER. If not, see <http://www.gnu.org/licenses/>.
*/

#include <data_structure/base/polytope.h>
#include <random>

using namespace geometry_utils;
using namespace color_text;
using namespace std;


Polytope::Polytope(MatD4f _planes) {
    planes = _planes;
    undefined = false;
}

bool Polytope::empty() const {
    return undefined;
}

bool Polytope::HaveSeedLine() const  {
    return have_seed_line;
}

void Polytope::SetSeedLine(const std::pair<Vec3f, Vec3f> &_seed_line, double r) {
    robot_r = r;
    seed_line = _seed_line;
    have_seed_line = true;
}

int Polytope::SurfNum() const {
    if (undefined) {
        return 0;
    }
    return planes.rows();
}

Vec3f Polytope::CrossCenter(const Polytope &b) const {
    MatD4f curIH;
    curIH.resize(this->SurfNum() + b.SurfNum(), 4);
    curIH << this->planes, b.GetPlanes();
    Mat3Df curIV; // 走廊的顶点
    if (!geometry_utils::enumerateVs(curIH, curIV)) {
        printf(" -- [processCorridor] Failed to get Overlap enumerateVs .\n");
        return Vec3f(-999, -999, -999);
    }
    double x = (curIV.row(0).maxCoeff() + curIV.row(0).minCoeff()) * 0.5;
    double y = (curIV.row(1).maxCoeff() + curIV.row(1).minCoeff()) * 0.5;
    double z = (curIV.row(2).maxCoeff() + curIV.row(2).minCoeff()) * 0.5;
    return Vec3f(x, y, z);
}


Polytope Polytope::CrossWith(const Polytope &b) const {
    MatD4f curIH;
    curIH.resize(this->SurfNum() + b.SurfNum(), 4);
    curIH << this->planes, b.GetPlanes();
    Polytope out;
    out.SetPlanes(curIH);
    return out;
}

bool Polytope::HaveOverlapWith(Polytope cmp, double eps) {
    return geometry_utils::overlap(this->planes, cmp.GetPlanes(), eps);
}


MatD4f Polytope::GetPlanes() const {
    return planes;
}

void Polytope::Reset() {
    undefined = true;
    is_known_free = false;
    have_seed_line = false;
}

bool Polytope::IsKnownFree() {
    if (undefined) {
        return false;
    }
    return is_known_free;
}

void Polytope::SetKnownFree(bool is_free) {
    is_known_free = is_free;
}

void Polytope::SetPlanes(MatD4f _planes) {
    planes = _planes;
    undefined = false;
}

void Polytope::SetEllipsoid(const Ellipsoid &ellip) {
    ellipsoid_ = ellip;
}

bool Polytope::PointIsInside(const Vec3f &pt, const double & margin) const {
    if (undefined) {
        return false;
    }
    if (planes.rows() == 0 || isnan(planes.sum())) {
        std::cout << YELLOW << "ill polytope, force return." << RESET << std::endl;
    }
    Eigen::Vector4d pt_e;
    pt_e.head(3) = pt;
    pt_e(3) = 1;
    if ((planes * pt_e).maxCoeff() > margin) {
        return false;
    }
    return true;
}

double Polytope::GetVolume() const {
    // 首先，我们需要获取多面体的顶点
    Eigen::Matrix<double, 3, -1, Eigen::ColMajor> vPoly;
    MatD4f planes = GetPlanes();
    if (!geometry_utils::enumerateVs(planes, vPoly)) {
        printf("Failed to compute volume: cannot enumerate vertices.\n");
        return 0;
    }

    // 使用QuickHull库计算凸包
    geometry_utils::QuickHull<double> qh;
    const auto convexHull = qh.getConvexHull(vPoly.data(), vPoly.cols(), false, true);
    const auto &indexBuffer = convexHull.getIndexBuffer();

    // 确保我们至少有四个顶点，这样才能构成一个四面体
    if (indexBuffer.size() < 4) {
        printf("Not enough vertices to compute volume.\n");
        return 0;
    }

    // 计算多面体的重心
    Vec3f centroid = Vec3f::Zero();
    for (long int i = 0; i < vPoly.cols(); i++) {
        centroid += vPoly.col(i);
    }
    centroid /= static_cast<double>(vPoly.cols());

    // 对每个三角形面，计算其四面体相对于重心的体积
    double volume = 0.0;
    for (size_t i = 0; i < indexBuffer.size(); i += 3) {
        Vec3f A = vPoly.col(indexBuffer[i]);
        Vec3f B = vPoly.col(indexBuffer[i + 1]);
        Vec3f C = vPoly.col(indexBuffer[i + 2]);

        // 使用标量三重积计算四面体的有向体积
        double tetrahedronVolume = (A - centroid).dot((B - centroid).cross(C - centroid)) / 6.0;
        volume += std::abs(tetrahedronVolume);
    }

    return volume;
}
