module FactoryRendering

using CoordinateTransformations
using Rotations
using GeometryBasics
using StaticArrays
using Parameters
using Compose
using Colors

export
    OrthographicProjection,
    BirdsEyeProjection,
    construct_unit_box,
    EntityRenderModel,
    get_render_layer

const DEFAULT_VTX_COLOR = colorant"LightGray"
default_vtx_color() = DEFAULT_VTX_COLOR
const DEFAULT_ENTITY_RADIUS = 0.5
const DEFAULT_ROBOT_RADIUS = 0.5
const DEFAULT_OBJECT_RADIUS = 0.25
default_entity_radius() = DEFAULT_ENTITY_RADIUS
default_robot_radius() = DEFAULT_ROBOT_RADIUS
default_object_radius() = DEFAULT_OBJECT_RADIUS
default_entity_shape() = Circle(Point(0.0,0.0),default_entity_radius())
const DEFAULT_ROBOT_COLOR = RGB(0.1,0.5,0.9)
const DEFAULT_OBJECT_COLOR = colorant"black"
default_robot_color() = DEFAULT_ROBOT_COLOR
default_object_color() = DEFAULT_OBJECT_COLOR
get_aspect_ratio() = Compose.default_graphic_height / Compose.default_graphic_width

abstract type RenderProjection end
(p::RenderProjection)(v::Vector) = map(p,v)
struct BirdsEyeProjection <: RenderProjection end
struct OrthographicProjection <: RenderProjection
    theta::Float64 # yaw
    azimuth::Float64 # roll
    lmap::LinearMap
end
function OrthographicProjection(theta,azimuth)
    tilt = LinearMap(SMatrix{2,3,Float64}(
        1.0, 0.0, 0.0, sin(azimuth), 0.0, 0.0))
    rot = LinearMap(RotZ(theta))
    OrthographicProjection(theta,azimuth,tilt ∘ rot)
end
function CoordinateTransformations.LinearMap(proj::OrthographicProjection) 
    proj.lmap
end

struct AxisAlignedEllipse <: GeometryPrimitive{2,Float64}
    center::Point2{Float64}
    rx::Float64
    ry::Float64
end
Compose.circle(c::Circle)               = Compose.circle(c.center[1],c.center[2],c.r)
Compose.ellipse(e::AxisAlignedEllipse) = Compose.ellipse(e.center[1],e.center[2],e.rx,e.ry)
Compose.polygon(n::GeometryBasics.Ngon) = Compose.polygon(Vector(map(NTuple{2,Float64}, coordinates(n))))
convert_to_compose_form(e::Circle)              = Compose.circle(e)
convert_to_compose_form(e::AxisAlignedEllipse)  = Compose.ellipse(e)
convert_to_compose_form(e::GeometryBasics.Ngon) = Compose.polygon(e)
Base.:(+)(c::Union{HyperSphere,Rect2D,GeometryBasics.Ngon,AxisAlignedEllipse},v::Vector) = map(p->c+p,v)
Base.:(+)(c::HyperSphere,p::Point2) = Circle(c.center+p,c.r)
Base.:(+)(c::AxisAlignedEllipse,p::Point2) = AxisAlignedEllipse(c.center+p,c.rx,c.ry)
function Base.:(+)(g::G,p::P) where {N,M,T,P<:Point{N,T},G<:GeometryBasics.Ngon{N,T,M,P}}
    G(map(c->c+p, coordinates(g)))
end

(proj::BirdsEyeProjection)(c::Point3{Float64}) = Point2{Float64}(c[1],c[2])
(proj::BirdsEyeProjection)(c::Point2{Float64}) = c
(proj::BirdsEyeProjection)(c::Circle) = c
(proj::BirdsEyeProjection)(c::Rect2D) = c
(proj::BirdsEyeProjection)(r::G) where {M,G<:GeometryBasics.Ngon{2,Float64,M,Point{2,Float64}}} = r

(proj::OrthographicProjection)(c::Point3{Float64}) = proj.lmap(c)
(proj::OrthographicProjection)(c::Point2{Float64}) = proj.lmap(Point(c[1],c[2],0.0))

function (proj::OrthographicProjection)(c::Circle)
    AxisAlignedEllipse(
        proj(Point(c.center[1],c.center[2],0.0)),
        c.r,
        c.r*sin(proj.azimuth)
    )
end
# function (proj::OrthographicProjection)(r::G) where {N,M,G<:GeometryBasics.Ngon{N,Float64,M,Point{N,Float64}}}
function (proj::RenderProjection)(r::G) where {N,M,G<:GeometryBasics.Ngon{N,Float64,M,Point{N,Float64}}}
    GeometryBasics.Ngon(SVector{M,Point2{Float64}}(map(proj,r.points)...))
end
function (proj::OrthographicProjection)(r::Rect2D)
    pts = map(Point2,collect(coordinates(r)))
    proj(GeometryBasics.Ngon(SVector(pts[1],pts[2],pts[3],pts[4])))
end

function bounding_hyperrect(vtxs::Vector{Point{N,Float64}},buffer=zero(Vec{N,Float64})) where {N}
    p1 = Vec{N,Float64}(map(i->minimum(map(v->v[i],vtxs)),1:N) .- buffer)
    p2 = Vec{N,Float64}(map(i->maximum(map(v->v[i],vtxs)),1:N) .+ 2*buffer)
    rect = Rect(p1, p2 .- p1)
end
function construct_unit_box(bbox,aspect=get_aspect_ratio())
    corners = collect(coordinates(bbox))
    xl = minimum(map(c->c[1],corners))
    yl = minimum(map(c->c[2],corners))
    xh = maximum(map(c->c[1],corners))
    yh = maximum(map(c->c[2],corners))
    construct_unit_box(xl,yl,xh,yh,aspect)
end
function construct_unit_box(xl,yl,xh,yh,aspect=get_aspect_ratio())
    @assert (xl < xh) "$xl >= $xh, but x_low must be less than x_high"
    @assert (yl < yh) "$yl >= $yh, but y_low must be less than y_high"
    dx = xh-xl
    dy = yh-yl
    if dy/dx > aspect
        dx = dy/aspect
    else
        dy = dx*aspect
    end
    cx = (1/2)*(xh+xl)
    cy = (1/2)*(yh+yl)
    UnitBox(cx-dx/2,cy-dy/2,dx,dy)
end

"""
    EntityConfiguration{N}

3D Robot or object configuration.
"""
@with_kw mutable struct EntityConfiguration
    pos::Point{3,Float64}   = zero(Point{3,Float64})
    theta::Float64          = 0.0
end
struct RenderPrimitive{G,C}
    geom::G
    color::C
end
const RenderStack{N} = NTuple{N,RenderPrimitive} 
function circle_entity_render_stack(radius,color)
    RenderStack{1}([
        # RenderPrimitive(Line(Point2(0.0,0.0),Point2(0.0,0.0)),color),
        RenderPrimitive(Circle(Point2(0.0,0.0),radius),color),
    ])
end
circle_robot_render_stack(radius=default_robot_radius(),color=default_robot_color()) = circle_entity_render_stack(radius,color)
circle_object_render_stack(radius=default_object_radius(),color=default_object_color()) = circle_entity_render_stack(radius,color)

# """
#     struct GridWorldModel{N}

# Models the layout of vertices on a grid floor
# """
# @with_kw mutable struct GridWorldModel{N,S}
#     vtxs::Vector{Point{N,Float64}}  = Vector{Point{N,Float64}}()
#     vtx_shape::S                    = Rect(zero(Vec{N,Float64}),0.45*Vec2(1.0,1.0))
#     vtx_colors::Vector{Color}       = Vector{Color}(map(v->default_vtx_color(),vtxs))
# end
# (t::AffineMap)(model::GridWorldModel) = GridWorldModel(
#         vtxs=map(t,model.vtxs),
#         vtx_shape=t(model.vtx_shape),
#         model.vtx_colors
#         )

"""
    mutable struct EntityRenderModel{G,C}

For rendering multiple entities. Fields:
*configs
*shapes
*colors (can be be a vector of colors or a single color)
"""
@with_kw mutable struct EntityRenderModel{N,G,C}
    configs::Vector{Point{N,Float64}}   = Vector{Point{N,Float64}}()
    shapes::G                           = map(c->default_entity_shape(), configs)
    colors::C                           = map(v->default_robot_color(),configs)
    label_entities::Bool                = false
end

function get_render_layer(m::EntityRenderModel,proj=BirdsEyeProjection())
    vtxs = proj(m.configs)
    shapes = proj(m.shapes)
    if isa(shapes,Vector)
        tformed_shapes = shapes .+ vtxs
    else
        tformed_shapes = shapes + vtxs
    end
    forms = map(convert_to_compose_form, tformed_shapes)
    return (context(), forms..., fill(m.colors))
end
function construct_unit_box(m::EntityRenderModel,proj=BirdsEyeProjection(),buffer=zeros(2))
    vtxs = map(proj, m.configs)
    construct_unit_box(bounding_hyperrect(vtxs,buffer))
end

end
