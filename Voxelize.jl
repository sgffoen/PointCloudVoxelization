using StaticArrays
using Glob
using LasIO
using FileIO


const VoxelId = NTuple{3, Int}

"""Read LAS file"""
function readPointcloud(lasfile::String)
    header,laspoints = load(lasfile) #LasIO
    npts = length(laspoints)
    #cloudarray = Array{Array{Float64,1},1}(npts)
    pointcloud_array = Vector{SVector{5, Real}}(npts)
    for (i,p) in enumerate(laspoints)
        #x, y, z, class, in = xcoord(p,header), ycoord(p,header), zcoord(p,header), classification(p), intensity(p)
        point_info = SVector{5, Real}(xcoord(p,header), ycoord(p,header), zcoord(p,header), classification(p), intensity(p))
        pointcloud_array[i] = point_info

        #push!(cloudarray,[x,y,z,class,in])
    end
    println("Reading las file complete")
    pointcloud_array
end


"""Type raster grid"""
immutable VoxelGrid{T <: Real}
    voxel_size::T
    boolean_grid::BitArray
    point_indices::Dict{VoxelId, Vector{Int}}
end


function RasterGrid3D{T1 <: AbstractVector, T2 <: Real}(points::Vector{T1}, voxel_size::T2)
    npoints = length(points)
    println("Pointcloud has $npoints points")
    # Generate Bounding Box
    MinBound, MaxBound = BoundingBox(points, voxel_size) # tuple of (xmin,ymin,zmin),(xmax,ymax,zmax)
    # compute number of voxels C in X, Y, Z direction
    Nx, Ny, Nz = (MaxBound[1]-MinBound[1]) / voxel_size, (MaxBound[2]-MinBound[2]) / voxel_size, (MaxBound[3]-MinBound[3]) / voxel_size
    println("number of voxels = $Nx x $Ny x $Nz")
    # Initialize Multi-D boolean array of size i, j, k with number voxels Nx, Ny, Nz
    BooleanArray = falses(floor(Int,Nx),floor(Int,Ny),floor(Int,Nz))
    # Initialize new list of 3D points
    VoxelsOut = Vector{SVector{3, Real}}(npoints)

    voxel_points= Dict{VoxelId, Vector{Int}}()

    # compute integer i,j,k for each point in pointcloud (i,j,k are coordinates of raster starting from 1)
    for (index, point) in enumerate(points)
        i = floor(Int,(point[1] - MinBound[1]) / voxel_size) +1 # Julia indexing starts at 1
        j = floor(Int,(point[2] - MinBound[2]) / voxel_size) +1
        k = floor(Int,(point[3] - MinBound[3]) / voxel_size) +1
        if BooleanArray[i,j,k] == false
            BooleanArray[i,j,k] = true # mark as visited
            # translate i,j,k back to coordinates of point cloud
            #VoxelOut = SVector{3, Real}(i * voxel_size, j * voxel_size, k * voxel_size)
            # voxel centroid
            #VoxelOut = VoxelOut + [voxel_size/2,voxel_size/2,voxel_size/2] + [MinBound[1], MinBound[2], MinBound[3]] - [1,1,1]
            #VoxelsOut[index] = VoxelOut # add voxel to ouput voxel list
            voxel_points[(i,j,k)] = [index]

        elseif BooleanArray[i,j,k] == true
            point_indices = voxel_points[(i,j,k)]
            voxel_points[(i,j,k)] = push!(point_indices, index)
        end
    end
    println("3D raster grid created")
    return VoxelGrid(voxel_size, BooleanArray, voxel_points)
end


function BoundingBox{T1 <: AbstractVector, T2 <: Real}(points::Vector{T1}, voxel_size::T2)
    npoints = length(points)
    x_coords, y_coords, z_coords = Vector{Real}(npoints), Vector{Real}(npoints), Vector{Real}(npoints)
    for j in 1:npoints
        x_coords[j], y_coords[j], z_coords[j] = points[j][1], points[j][2], points[j][3]
    end
    # minimum/maximum coordinates (x,y,z)
    min_coords = (minimum(x_coords), minimum(y_coords), minimum(z_coords))
    max_coords = (maximum(x_coords), maximum(y_coords), maximum(z_coords))

    # Defines the bounding box corresponding to the voxels
    MinBound = (fld(min_coords[1], voxel_size)*voxel_size, fld(min_coords[2], voxel_size)*voxel_size, fld(min_coords[3], voxel_size)*voxel_size)
    MaxBound = (cld(max_coords[1], voxel_size)*voxel_size, cld(max_coords[2], voxel_size)*voxel_size, cld(max_coords[3], voxel_size)*voxel_size)

    return(MinBound, MaxBound)
end


""" Run code here """

"""
# Example
To create a spatial grid with voxel side length of 2.0 metres with random points:
    
    points = rand(3, 1000) * 20.0
    grid = RasterGrid3D(points, 2.0)




# read point cloud (.las/.laz) from memory
workdir = dirname(@__FILE__)
globstring = "*.laz"
lasfiles = glob(globstring, workdir)

@show lasfiles
println(lasfiles)



@time pointcloud = readPointcloud(lasfiles[1])
# set voxelsize
voxelsize = 2.0

@time voxelsgrid = RasterGrid3D(pointcloud, voxelsize) 

""" 