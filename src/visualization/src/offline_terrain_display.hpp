#pragma once
// Offline rendering formulas. Input meshes are already in the world frame.
#include <open3d/Open3D.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <future>
#include <set>
namespace offline_terrain {
namespace geometry = open3d::geometry;
using Mesh = std::shared_ptr<geometry::TriangleMesh>;
using Pose = Eigen::Matrix4d;
// Fixed-light CPU rasterizer for reproducible offline comparison over SSH.
static geometry::Image raster(const std::vector<Mesh>& meshes, const Eigen::Vector3d& eye,
                              const Eigen::Vector3d& front, const Eigen::Vector3d& up,
                              bool provenance=false) {
    constexpr int width=800, height=600;
    constexpr double focal=519.615242;
    const Eigen::Vector3d right=up.cross(front).normalized();
    const Eigen::Vector3d light=Eigen::Vector3d(-.4,-.5,1).normalized();
    geometry::Image image;
    image.Prepare(width,height,3,1);
    std::fill(image.data_.begin(),image.data_.end(),26);
    std::vector<float> depth(width*height,0);
    struct Vertex { Eigen::Vector2d p; double inverse_z; };
    const auto edge=[](const Eigen::Vector2d& a,const Eigen::Vector2d& b,double x,double y) {
        return (b.x()-a.x())*(y-a.y())-(b.y()-a.y())*(x-a.x());
    };
    for(const auto& mesh:meshes) {
        std::vector<Vertex> projected;
        projected.reserve(mesh->vertices_.size());
        for(const auto& point:mesh->vertices_) {
            const Eigen::Vector3d delta=point-eye;
            const double z=-delta.dot(front);
            if(z<=1 || z>=20000) projected.push_back({Eigen::Vector2d::Zero(),0});
            else projected.push_back({{width/2+focal*delta.dot(right)/z,height/2-focal*delta.dot(up)/z},1/z});
        }
        for(const auto& face:mesh->triangles_) {
            const auto& a=projected[face.x()];
            const auto& b=projected[face.y()];
            const auto& c=projected[face.z()];
            if(a.inverse_z==0 || b.inverse_z==0 || c.inverse_z==0) continue;
            const double area=edge(a.p,b.p,c.p.x(),c.p.y());
            if(std::abs(area)<1e-8) continue;
            const int xmin=static_cast<int>(std::max(0.,std::floor(std::min({a.p.x(),b.p.x(),c.p.x()}))));
            const int xmax=static_cast<int>(std::min(width-1.,std::ceil(std::max({a.p.x(),b.p.x(),c.p.x()}))));
            const int ymin=static_cast<int>(std::max(0.,std::floor(std::min({a.p.y(),b.p.y(),c.p.y()}))));
            const int ymax=static_cast<int>(std::min(height-1.,std::ceil(std::max({a.p.y(),b.p.y(),c.p.y()}))));
            const Eigen::Vector3d normal=(mesh->vertices_[face.y()]-mesh->vertices_[face.x()]).cross(
                mesh->vertices_[face.z()]-mesh->vertices_[face.x()]).normalized();
            const auto color=static_cast<uint8_t>(100+120*std::abs(normal.dot(light)));
            for(int y=ymin;y<=ymax;++y) for(int x=xmin;x<=xmax;++x) {
                const double wa=edge(b.p,c.p,x+.5,y+.5)/area;
                const double wb=edge(c.p,a.p,x+.5,y+.5)/area;
                const double wc=1-wa-wb;
                if(wa<0 || wb<0 || wc<0) continue;
                const float z=wa*a.inverse_z+wb*b.inverse_z+wc*c.inverse_z;
                const auto index=y*width+x;
                if(z<=depth[index]) continue;
                depth[index]=z;
                if(provenance && mesh->HasVertexColors()) {
                    const auto& rgb=mesh->vertex_colors_[face.x()];
                    for(int k=0;k<3;++k) image.data_[3*index+k]=static_cast<uint8_t>(color*rgb[k]);
                } else std::fill_n(image.data_.begin()+3*index,3,color);
            }
        }
    }
    return image;
}

struct Coverage {
    std::set<std::pair<int64_t,int64_t>> occupied;
    Mesh cumulative=std::make_shared<geometry::TriangleMesh>();
    static constexpr double cell=5.;
    static auto key(const Eigen::Vector2d& p) {
        return std::make_pair(int64_t(std::floor(p.x()/cell)),int64_t(std::floor(p.y()/cell)));
    }
    Mesh append(const Mesh& local,const Pose& pose) {
        auto world=std::make_shared<geometry::TriangleMesh>(*local);
        world->Transform(pose);
        std::vector<Eigen::Vector3i> added;
        // Test all triangles against the previous snapshot, then rasterize the batch.
        for(const auto& face:world->triangles_) {
            const Eigen::Vector2d a=world->vertices_[face[0]].head<2>();
            const Eigen::Vector2d b=world->vertices_[face[1]].head<2>();
            const Eigen::Vector2d c=world->vertices_[face[2]].head<2>();
            const std::array<Eigen::Vector2d,7> samples{a,b,c,(a+b+c)/3.,(a+b)/2.,(b+c)/2.,(c+a)/2.};
            bool keep=true;
            for(const auto& p:samples) if(occupied.count(key(p))) {keep=false;break;}
            if(keep) added.push_back(face);
        }
        for(const auto& face:added) {
            const Eigen::Vector2d a=world->vertices_[face[0]].head<2>()/cell;
            const Eigen::Vector2d b=world->vertices_[face[1]].head<2>()/cell;
            const Eigen::Vector2d c=world->vertices_[face[2]].head<2>()/cell;
            const Eigen::Vector2d u=b-a,v=c-a;
            const double det=u.x()*v.y()-u.y()*v.x();
            if(std::abs(det)<1e-12) continue;
            const auto lo=a.cwiseMin(b).cwiseMin(c).array().floor().cast<int64_t>().eval();
            const auto hi=a.cwiseMax(b).cwiseMax(c).array().ceil().cast<int64_t>().eval();
            for(int64_t y=lo[1];y<=hi[1];++y) for(int64_t x=lo[0];x<=hi[0];++x) {
                const double dx=x+.5-a.x(),dy=y+.5-a.y();
                const double w=(dx*v.y()-dy*v.x())/det,z=(u.x()*dy-u.y()*dx)/det;
                if(w>=0 && z>=0 && w+z<=1) occupied.emplace(x,y);
            }
        }
        const int offset=static_cast<int>(cumulative->vertices_.size());
        cumulative->vertices_.insert(cumulative->vertices_.end(),world->vertices_.begin(),world->vertices_.end());
        for(auto face:added) cumulative->triangles_.push_back(face.array()+offset);
        return std::make_shared<geometry::TriangleMesh>(*cumulative);
    }
};

struct SceneFrame {
    Pose nav=Pose::Identity();
    Mesh update, cumulative;
};

struct Segment {
    Eigen::Vector2d desired;
    double yaw0=0,turn=0;
    Segment(const SceneFrame& a,const SceneFrame& b,Eigen::Vector2d heading,bool first) {
        desired=(b.nav.block<3,1>(0,3)-a.nav.block<3,1>(0,3)).head<2>();
        if(desired.norm()>1e-6) desired.normalize(); else desired=heading;
        if(first) heading=desired;
        yaw0=std::atan2(heading.y(),heading.x());
        turn=std::remainder(std::atan2(desired.y(),desired.x())-yaw0,2*M_PI);
    }
    geometry::Image draw(const SceneFrame& a,const SceneFrame& b,double u) const {
        const double yaw=yaw0+turn*u*u*(3-2*u);
        const Eigen::Vector3d g(std::cos(yaw),std::sin(yaw),0);
        const Eigen::Vector3d P=(1-u)*a.nav.block<3,1>(0,3)+u*b.nav.block<3,1>(0,3);
        const double s=.72,c=std::sqrt(1-s*s);
        const Eigen::Vector3d front=-c*g+s*Eigen::Vector3d::UnitZ();
        const Eigen::Vector3d up=s*g+c*Eigen::Vector3d::UnitZ();
        const Eigen::Vector3d eye=P+400*g+680*front;
        // Independent immutable layers; preserve all arithmetic within each raster.
        // This changes scheduling only, not depth ordering or composition.
        auto recent_future=std::async(std::launch::async,[&]{return raster({b.update},eye,front,up);});
        auto older_future=std::async(std::launch::async,[&]{return raster({a.update},eye,front,up);});
        auto image=raster({b.cumulative},eye,front,up);
        const auto recent=recent_future.get();
        const auto older=older_future.get();
        const double t=std::min(1.,u*2),fade=t*t*(3-2*t);
        for(int y=0;y<160;++y) for(int x=0;x<800;++x) {
            const size_t p=3*(800*y+x);
            const double edge=std::min(1.,(160-y)/65.);
            const double wa=edge*(1-fade)*(older.data_[p]!=26);
            const double wb=edge*fade*(recent.data_[p]!=26);
            for(int ch=0;ch<3;++ch) image.data_[p+ch]=static_cast<uint8_t>(
                (1-wa-wb)*image.data_[p+ch]+wa*older.data_[p+ch]+wb*recent.data_[p+ch]);
        }
        return image;
    }
};

} // namespace offline_terrain
