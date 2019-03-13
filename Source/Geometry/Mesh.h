//
// Created by pierre on 12/03/19.
//

#ifndef SIMPLE_RAYTRACER_MESH_H
#define SIMPLE_RAYTRACER_MESH_H

#include "SceneObject.h"

#include <igl/readOFF.h>
#include <igl/readOBJ.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/signed_distance.h>

class Mesh  : public SceneObject {
public:
    Mesh(const std::string& path, const Vec3f& center, const Material& material)
            : SceneObject(material), m_path(path), m_center(center) {
        Eigen::MatrixXd rawV;
        if (igl::readOFF(m_path, rawV, _F)) {

            Eigen::MatrixXd minRow = rawV.colwise().minCoeff();
            Eigen::MatrixXd maxRow = rawV.colwise().maxCoeff();
            Eigen::MatrixXd barycenter = rawV.colwise().mean();
            Eigen::MatrixXd boxSize = maxRow - minRow;

            Eigen::MatrixXd eCenter(1, 3);
            eCenter.row(0) << center.x(), center.y(), center.z();
            _V = rawV;
            _V.rowwise() -= (barycenter.row(0) - eCenter.row(0));
//            _V.array().rowwise() /= boxSize.row(0).array();
            _V /= boxSize.row(0).maxCoeff();

            _tree.init(_V, _F);
            igl::per_face_normals(_V, _F, _FN);
            igl::per_vertex_normals(_V, _F,igl::PER_VERTEX_NORMALS_WEIGHTING_TYPE_ANGLE, _FN, _VN);
            igl::per_edge_normals(_V, _F,igl::PER_EDGE_NORMALS_WEIGHTING_TYPE_UNIFORM, _FN, _EN, _E, _EMAP);
        }
    }
    virtual void sdf (IN const Vec3f& position, OUT Intersection& output_intersection) const override;
private:
    std::string m_path;
    Eigen::MatrixXd _V;
    Eigen::MatrixXi _F;
    igl::AABB<Eigen::MatrixXd,3> _tree;
    Eigen::MatrixXd _FN;
    Eigen::MatrixXd _VN;
    Eigen::MatrixXd _EN;
    Eigen::MatrixXi _E;
    Eigen::VectorXi _EMAP;
    Vec3f m_center;
};


#endif //SIMPLE_RAYTRACER_MESH_H
