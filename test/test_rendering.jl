import Cairo
using Compose
using Colors
using GeometryBasics
using StaticArrays
using FactoryRendering


# rendering practice
Compose.set_default_graphic_size(20cm,20cm)
# vtxs coordinates and shapes
vtx_shape = GeometryBasics.Ngon(SVector(map(i->0.6*Point(sin(i*π/2+π/4),cos(i*π/2+π/4),0.0),0:3)...))
vtxs = Vector{Point3{Float64}}()
s = [15,20]
for i in 1:s[1]
    for j in 1:s[2]
        push!(vtxs, Point3{Float64}(i,j,0.0))
    end
end
floor_model = EntityRenderModel(
    configs=vtxs,
    shapes = map(v->vtx_shape,vtxs),
    colors = HSVA(50,0,1,0.2),
    )
# robot coordinates and shapes
r_vtxs = vtxs[[1,28,137]]
r_shape = Circle(Point(0.0,0.0),0.3)
robot_model = FactoryRendering.EntityRenderModel(
    configs=r_vtxs,
    shapes = map(v->r_shape,r_vtxs),
    colors = map(v->HSVA(200,1,1,0.9),r_vtxs)
)
o_vtxs=vtxs[[28,137,41,267]]
object_model = FactoryRendering.EntityRenderModel(
    configs=o_vtxs,
    shapes = Circle(Point(0.0,0.0),0.15),
    colors = HSVA(0,0,0,0.9)
)
# OrthographicProjection
theta = π/3
azimuth = π/8
proj = FactoryRendering.OrthographicProjection(theta,azimuth)
# proj = FactoryRendering.BirdsEyeProjection()

unit_box = FactoryRendering.construct_unit_box(floor_model,proj,ones(2))
Compose.compose(context(units=unit_box),
    get_render_layer(object_model,proj),
    get_render_layer(robot_model,proj),
    get_render_layer(floor_model,proj),
    )
