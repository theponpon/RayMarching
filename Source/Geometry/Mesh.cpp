//
// Created by pierre on 12/03/19.
//

#include "Mesh.h"

#include <torch/torch.h>

void Mesh::sdf(IN const Vec3f& position, OUT Intersection& output_intersection) const {

    Eigen::VectorXi I;
    Eigen::MatrixXd N,C;
    Eigen::MatrixXd V_ray(1, 3), S_ray;

    Eigen::Vector3d centeredPos = _orientation * (Eigen::Vector3d(position.x(), position.y(), position.z()) - m_center);
    V_ray.row(0) = centeredPos;

    signed_distance_pseudonormal(V_ray, _V, _F, _tree, _FN, _VN, _EN, _EMAP, S_ray, I, C, N);

//    igl::SignedDistanceType sign_type = igl::SIGNED_DISTANCE_TYPE_DEFAULT;
//    igl::signed_distance(V_ray, _V, _F,sign_type, S_ray,I,C,N);

    output_intersection = Intersection(S_ray(0), surface_material(), position);

    torch::Tensor tensor = torch::eye(3);

    return;
}