//
// Created by pierre on 12/03/19.
//

#include "Mesh.h"

void Mesh::sdf(IN const Vec3f& position, OUT Intersection& output_intersection) const {

    Eigen::VectorXi I;
    Eigen::MatrixXd N,C;
    Eigen::MatrixXd V_ray(1, 3), S_ray;

    Vec3f sample_point = position + m_center;
    V_ray.row(0) << sample_point.x(), sample_point.y(), sample_point.z();

    signed_distance_pseudonormal(V_ray, _V, _F, _tree, _FN, _VN, _EN, _EMAP, S_ray, I, C, N);

//    igl::SignedDistanceType sign_type = igl::SIGNED_DISTANCE_TYPE_DEFAULT;
//    igl::signed_distance(V_ray, _V, _F,sign_type, S_ray,I,C,N);

    output_intersection = Intersection(S_ray(0), surface_material(), position);

    return;
}