import {mat3d, quat4, Quat4, vec2d, Vec2d, vec3d, Vec3d, Vec3} from "gl-matrix";

export interface Obb {
  center: Vec3d;
  halfSize: Vec3;
  quaternion: Quat4;
}

export interface Attribute {
  /** Data uses the data type of the vertex attribute */
  data: WritableArrayLike<number>;
  /** Components per vertex */
  size: number;

  /** Index into data array i.e. not a byte offset  */
  offsetIdx: number;
  /** Stride across data array i.e. not a byte stride */
  strideIdx: number;
}

const epsilon: number = 0.000001;
const alMid = vec3d.create();
const alLen = vec3d.create();

// Derived from the C++ sample implementation of http://www.idt.mdh.se/~tla/publ/FastOBBs.pdf
export function computeOBB(positions: Attribute, obb: Obb) {

  const {data, strideIdx} = positions;
  const count = data.length / strideIdx;
  if (count <= 0) {
    return;
  }

  // Select seven extremal points along predefined slab directions
  const extremals: ExtremalPoints = new ExtremalPoints(positions);

  // Compute size of AABB (max and min projections of vertices are already
  // computed as slabs 0-2)
  vec3d.add(extremals.minProj, extremals.maxProj, alMid);
  vec3d.scale(alMid, 0.5);

  vec3d.subtract(extremals.maxProj, extremals.minProj, alLen);

  const alVal = _getQualityValue(alLen);
  const best = new Orientation();
  best.quality = alVal;

  if (count < 14) {
    positions = {data: new Float64Array(extremals.buffer, 14 * 8, 14 * 3), size: 3, offsetIdx: 0, strideIdx: 3 };
  }

  // Find best OBB axes based on the constructed base triangle
  const p0 = vec3d.create(); // Vertices of the large base triangle
  const p1 = vec3d.create(); // Vertices of the large base triangle
  const p2 = vec3d.create(); // Vertices of the large base triangle
  const e0 = vec3d.create(); // Edge vectors of the large base triangle
  const e1 = vec3d.create(); // Edge vectors of the large base triangle
  const e2 = vec3d.create(); // Edge vectors of the large base triangle
  const n = vec3d.create(); // Unit normal of the large base triangle

  switch (_findBestObbAxesFromBaseTriangle(extremals, positions, n, p0, p1, p2, e0, e1, e2, best, obb)) {
     case 1:
       _finalizeAxisAlignedOBB(alMid, alLen, obb);
       return;
     case 2:
       _finalizeLineAlignedOBB(positions, e0, obb);
       return;
  }

  // Find improved OBB axes based on constructed di-tetrahedral shape raised from base triangle
  _findImprovedObbAxesFromUpperAndLowerTetrasOfBaseTriangle(positions, n, p0, p1, p2, e0, e1, e2, best, obb);

  // compute the true obb dimensions by iterating over all vertices
  _computeObbDimensions(positions, best.b0, best.b1, best.b2, bMin, bMax);
  const bLen = vec3d.create();
  vec3d.subtract(bMax, bMin, bLen);
  best.quality = _getQualityValue(bLen);

  // Check if the OBB extent is still smaller than the intial AABB
  if (best.quality < alVal) {
    _finalizeOBB(best.b0, best.b1, best.b2, bMin, bMax, bLen, obb); // if so, assign all OBB params
  }
  else {
    _finalizeAxisAlignedOBB(alMid, alLen, obb); // otherwise, assign all OBB params using the intial AABB
  }
}

function _findBestObbAxesFromBaseTriangle(extremals: ExtremalPoints, positions: Attribute, n: Vec3d,
  p0: Vec3d, p1: Vec3d, p2: Vec3d,
  e0: Vec3d, e1: Vec3d, e2: Vec3d,
  best: Orientation, obb: Obb ) {

    _findFurthestPointPair(extremals, p0, p1);

    // Degenerate case 1:
    // If the furthest points are very close, return OBB aligned with the initial AABB
    if (vec3d.dist2(p0, p1) < epsilon) {
      return 1;
    }

    // Compute edge vector of the line segment p0, p1
    vec3d.subtract(p0, p1, e0);
    vec3d.normalize(e0);

    // Find a third point furthest away from line given by p0, e0 to define the large base triangle
    const dist2 = _findFurthestPointFromInfiniteEdge(positions, p0, e0, p2);

    // Degenerate case 2:
    // If the third point is located very close to the line, return an OBB aligned with the line
    if (dist2 < epsilon) {
      return 2;
    }

    // Compute the two remaining edge vectors and the normal vector of the base triangle
    vec3d.subtract(p1, p2, e1);
    vec3d.normalize(e1);
    vec3d.subtract(p2, p0, e2);
    vec3d.normalize(e2);
    vec3d.cross(e1, e0, n);
    vec3d.normalize(n);

    _findBestObbAxesFromTriangleNormalAndEdgeVectors(positions, n, e0, e1, e2, best);
    return 0; // success
}

const q0 = vec3d.create();
const q1 = vec3d.create();
const f0 = vec3d.create(); // Edge vectors towards q0/1
const f1 = vec3d.create(); // Edge vectors towards q0/1
const f2 = vec3d.create(); // Edge vectors towards q0/1
const n0 = vec3d.create(); // Unit normals of tetra tris
const n1 = vec3d.create(); // Unit normals of tetra tris
const n2 = vec3d.create(); // Unit normals of tetra tris

function _findImprovedObbAxesFromUpperAndLowerTetrasOfBaseTriangle(positions: Attribute, n: Vec3d,
  p0: Vec3d, p1: Vec3d, p2: Vec3d,
  e0: Vec3d, e1: Vec3d, e2: Vec3d,
  best: Orientation, obb: Obb ) {

  // Find furthest points above and below the plane of the base triangle for tetra constructions
  _findUpperLowerTetraPoints(positions, n, p0, p1, p2, q0, q1);

  // For each valid point found, search for the best OBB axes based on the 3 arising triangles
  if (q0[0] !== undefined) {

    vec3d.subtract(q0, p0, f0);
    vec3d.normalize(f0);
    vec3d.subtract(q0, p1, f1);
    vec3d.normalize(f1);
    vec3d.subtract(q0, p2, f2);
    vec3d.normalize(f2);

    vec3d.cross(f1, e0, n0);
    vec3d.normalize(n0);
    vec3d.cross(f2, e1, n1);
    vec3d.normalize(n1);
    vec3d.cross(f0, e2, n2);
    vec3d.normalize(n2);

    _findBestObbAxesFromTriangleNormalAndEdgeVectors(positions, n0, e0, f1, f0, best);
    _findBestObbAxesFromTriangleNormalAndEdgeVectors(positions, n1, e1, f2, f1, best);
    _findBestObbAxesFromTriangleNormalAndEdgeVectors(positions, n2, e2, f0, f2, best);
  }
  if (q1[0] !== undefined) {

    vec3d.subtract(q1, p0, f0);
    vec3d.normalize(f0);
    vec3d.subtract(q1, p1, f1);
    vec3d.normalize(f1);
    vec3d.subtract(q1, p2, f2);
    vec3d.normalize(f2);

    vec3d.cross(f1, e0, n0);
    vec3d.normalize(n0);
    vec3d.cross(f2, e1, n1);
    vec3d.normalize(n1);
    vec3d.cross(f0, e2, n2);
    vec3d.normalize(n2);

    _findBestObbAxesFromTriangleNormalAndEdgeVectors(positions, n0, e0, f1, f0, best);
    _findBestObbAxesFromTriangleNormalAndEdgeVectors(positions, n1, e1, f2, f1, best);
    _findBestObbAxesFromTriangleNormalAndEdgeVectors(positions, n2, e2, f0, f2, best);
  }
}

function _findFurthestPointPair(extremals: ExtremalPoints, p0: Vec3d, p1: Vec3d) {

  let maxDist2 = vec3d.dist2(extremals.maxVert[0], extremals.minVert[0]);
  let index: number = 0;

  for (let i = 1; i < 7; ++i) {
    const dist2 = vec3d.dist2(extremals.maxVert[i], extremals.minVert[i]);
    if (dist2 > maxDist2) {
      maxDist2 = dist2;
      index = i;
    }
  }
  vec3d.set(extremals.minVert[index], p0);
  vec3d.set(extremals.maxVert[index], p1);
}

const u0 = vec3d.create();

function _findFurthestPointFromInfiniteEdge(positions: Attribute, p0: Vec3d, e0: Vec3d,
  p: Vec3d ): number {

  const {data, offsetIdx, strideIdx} = positions;

  let maxDist2 = Number.NEGATIVE_INFINITY;
  let maxIndex: number = 0;

  for (let i = offsetIdx; i < data.length; i += strideIdx) {
    // inlined _dist2PointInfiniteEdge
    vec3d.set3(data[i] - p0[0], data[i + 1] - p0[1], data[i + 2] - p0[2], u0);
    const t = e0[0] * u0[0] + e0[1] * u0[1] + e0[2] * u0[2];
    const sqLen_e0 = vec3d.length2(e0);
    const dist2 = vec3d.length2(u0) - t * t / sqLen_e0;

    if (dist2 > maxDist2) {
      maxDist2 = dist2;
      maxIndex = i;
    }
  }

  vec3d.set3(data[maxIndex], data[maxIndex + 1], data[maxIndex + 2], p);
  return maxDist2;
}

const minmax = vec2d.create();

function _findUpperLowerTetraPoints(positions: Attribute, n: Vec3d,
  p0: Vec3d, p1: Vec3d, p2: Vec3d,
  q0: Vec3d, q1: Vec3d) {

  _findExtremalPoints_OneDir(positions, n, minmax, q1, q0);
  const triProj = vec3d.dot(p0, n);

  if (minmax[1] - epsilon <= triProj) {
    q0[0] = undefined; // invalidate
  }
  if (minmax[0] + epsilon >= triProj) {
    q1[0] = undefined; // invalidate
  }
}

const m0 = vec3d.create();
const m1 = vec3d.create();
const m2 = vec3d.create();
const dmax = vec3d.create();
const dmin = vec3d.create();
const dlen = vec3d.create();

function _findBestObbAxesFromTriangleNormalAndEdgeVectors(positions: Attribute, n: Vec3d,
  e0: Vec3d, e1: Vec3d, e2: Vec3d, best: Orientation) {

  if (vec3d.length2(n) < epsilon) {
    return;
  }

  vec3d.cross(e0, n, m0);
  vec3d.cross(e1, n, m1);
  vec3d.cross(e2, n, m2);

  // The operands are assumed to be orthogonal and unit normals
  _findExtremalProjs_OneDir(positions, n, minmax);
  dmin[1] = minmax[0];
  dmax[1] = minmax[1];
  dlen[1] = dmax[1] - dmin[1];

  const edges = [e0, e1, e2];
  const ems = [m0, m1, m2];

  for (let i = 0; i < 3; ++i) {
    _findExtremalProjs_OneDir(positions, edges[i], minmax);
    dmin[0] = minmax[0];
    dmax[0] = minmax[1];

    _findExtremalProjs_OneDir(positions, ems[i], minmax);
    dmin[2] = minmax[0];
    dmax[2] = minmax[1];

    dlen[0] = dmax[0] - dmin[0];
    dlen[2] = dmax[2] - dmin[2];
    const quality = _getQualityValue(dlen);

    if (quality < best.quality) {
      vec3d.set(edges[i], best.b0);
      vec3d.set(n, best.b1);
      vec3d.set(ems[i], best.b2);
      best.quality = quality;
    }
  }
}

const point = vec3d.create();

function _findExtremalProjs_OneDir(positions: Attribute, n: Vec3d, minmax: Vec2d) {

  const {data, offsetIdx, strideIdx} = positions;

  minmax[0] = Number.POSITIVE_INFINITY;
  minmax[1] = Number.NEGATIVE_INFINITY;

  for (let i = offsetIdx; i < data.length; i += strideIdx) {

    const proj = data[i] * n[0] + data[i + 1] * n[1] + data[i + 2] * n[2]; // opt: inline dot product
    minmax[0] = Math.min(minmax[0], proj);
    minmax[1] = Math.max(minmax[1], proj);
  }
}

function _findExtremalPoints_OneDir(positions: Attribute, n: Vec3d, minmax: Vec2d,
   minVert: Vec3d, maxVert: Vec3d) {

  const {data, offsetIdx, strideIdx} = positions;
  vec3d.set3(data[offsetIdx], data[offsetIdx + 1], data[offsetIdx + 2], minVert);
  vec3d.set(minVert, maxVert);

  minmax[0] = vec3d.dot(point, n);
  minmax[1] = minmax[0];

  for (let i = offsetIdx + strideIdx; i < data.length; i += strideIdx) {

    vec3d.set3(data[i], data[i + 1], data[i + 2], point);
    const proj = vec3d.dot(point, n);

    if (proj < minmax[0]) {
        minmax[0] = proj;
        vec3d.set(point, minVert);
    }
    if (proj > minmax[1]) {
        minmax[1] = proj;
        vec3d.set(point, maxVert);
    }
  }
}

function _finalizeAxisAlignedOBB(mid: Vec3d, len: Vec3d, obb: Obb) {

  vec3d.set(mid, obb.center);
  vec3d.scale(len, 0.5, obb.halfSize);
  quat4.identity(obb.quaternion);
}

const r = vec3d.create();
const v = vec3d.create();
const w = vec3d.create();
const bMin = vec3d.create();
const bMax = vec3d.create();
const bLen = vec3d.create();

// This function is only called if the construction of the large base triangle fails
function _finalizeLineAlignedOBB(positions: Attribute, u: Vec3d, obb: Obb) {

  // Given u, build any orthonormal base u, v, w
  // Make sure r is not equal to u
  vec3d.set(u, r);
  if (Math.abs(u[0]) > Math.abs(u[1]) && Math.abs(u[0]) > Math.abs(u[2])) {
    r[0] = 0;
  }
  else if (Math.abs(u[1]) > Math.abs(u[2])) {
    r[1] = 0;
  }
  else     {
    r[2] = 0;
  }

  if (vec3d.length2(r) < epsilon) {
    r[0] = r[1] = r[2] = 1;
  }

  vec3d.cross(u, r, v);
  vec3d.normalize(v);
  vec3d.cross(u, v, w);
  vec3d.normalize(w);

  // compute the true obb dimensions by iterating over all vertices
  _computeObbDimensions(positions, u, v, w, bMin, bMax);
  vec3d.subtract(bMax, bMin, bLen);
  _finalizeOBB(u, v, w, bMin, bMax, bLen, obb);
}

function _computeObbDimensions(positions: Attribute, v0: Vec3d, v1: Vec3d, v2: Vec3d,
  min: Vec3d, max: Vec3d) {

  _findExtremalProjs_OneDir(positions, v0, minmax);
  min[0] = minmax[0];
  max[0] = minmax[1];
  _findExtremalProjs_OneDir(positions, v1, minmax);
  min[1] = minmax[0];
  max[1] = minmax[1];
  _findExtremalProjs_OneDir(positions, v2, minmax);
  min[2] = minmax[0];
  max[2] = minmax[1];
}

const tmp = vec3d.create();
const rot = mat3d.create();
const q = vec3d.create();

function _finalizeOBB(v0: Vec3d, v1: Vec3d, v2: Vec3d,
  min: Vec3d, max: Vec3d, len: Vec3d, obb: Obb) {

  rot[0] = v0[0]; rot[3] = v0[1]; rot[6] = v0[2];
  rot[1] = v1[0]; rot[4] = v1[1]; rot[7] = v1[2];
  rot[2] = v2[0]; rot[5] = v2[1]; rot[8] = v2[2];
  quat4.fromRotationMatrix(rot, obb.quaternion);

  // midpoint expressed in the OBB's own coordinate system
  vec3d.add(min, max, q);
  vec3d.scale(q, 0.5);

  // Compute midpoint expressed in the standard base
  vec3d.scale(v0, q[0], obb.center);
  vec3d.scale(v1, q[1], tmp);
  vec3d.add(obb.center, tmp);
  vec3d.scale(v2, q[2], tmp);
  vec3d.add(obb.center, tmp);

  vec3d.scale(len, 0.5, obb.halfSize);
}

class ExtremalPoints {
  buffer: ArrayBuffer;

  minProj: Float64Array;
  maxProj: Float64Array;
  minVert: Vec3d[] = [];
  maxVert: Vec3d[] = [];

  constructor(positions: Attribute) {

    // setup storage
    const numPoints = 7;
    const bufferSize = numPoints * (8 + 8 + 24 + 24);
    this.buffer = new ArrayBuffer(bufferSize);

    let offset: number = 0;
    this.minProj = new Float64Array(this.buffer, offset, numPoints);
    offset += numPoints * 8;

    this.maxProj = new Float64Array(this.buffer, offset, numPoints);
    offset += numPoints * 8;

    while (this.minVert.length < numPoints) {
      this.minVert.push(new Float64Array(this.buffer, offset, 3));
      offset += 24;
    }
    while (this.maxVert.length < numPoints) {
      this.maxVert.push(new Float64Array(this.buffer, offset, 3));
      offset += 24;
    }

    // init storage
    for (let i = 0; i < numPoints; ++i) {
      this.minProj[i] = Number.POSITIVE_INFINITY;
      this.maxProj[i] = Number.NEGATIVE_INFINITY;
    }
    const minIndices: number[] = [];
    const maxIndices: number[] = [];

    const {data, offsetIdx, strideIdx} = positions;

    // find extremal points
    for (let i = offsetIdx; i < data.length; i += strideIdx ) {

      // Slab 0: dir {1, 0, 0}
      let proj: number = data[i];
      if (proj < this.minProj[0]) {
        this.minProj[0] = proj;
        minIndices[0] = i;
      }
      if (proj > this.maxProj[0]) {
        this.maxProj[0] = proj;
        maxIndices[0] = i;
      }

      // Slab 1: dir {0, 1, 0}
      proj = data[i + 1];
      if (proj < this.minProj[1]) {
        this.minProj[1] = proj;
        minIndices[1] = i;
      }
      if (proj > this.maxProj[1]) {
        this.maxProj[1] = proj;
        maxIndices[1] = i;
      }

      // Slab 2: dir {0, 0, 1}
      proj = data[i + 2];
      if (proj < this.minProj[2]) {
        this.minProj[2] = proj;
        minIndices[2] = i;
      }
      if (proj > this.maxProj[2]) {
        this.maxProj[2] = proj;
        maxIndices[2] = i;
      }

      // Slab 3: dir {1, 1, 1}
      proj = data[i] + data[i + 1] + data[i + 2];
      if (proj < this.minProj[3]) {
        this.minProj[3] = proj;
        minIndices[3] = i;
      }
      if (proj > this.maxProj[3]) {
        this.maxProj[3] = proj;
        maxIndices[3] = i;
      }

      // Slab 4: dir {1, 1, -1}
      proj = data[i] + data[i + 1] - data[i + 2];
      if (proj < this.minProj[4]) {
        this.minProj[4] = proj;
        minIndices[4] = i;
      }
      if (proj > this.maxProj[4]) {
        this.maxProj[4] = proj;
        maxIndices[4] = i;
      }

      // Slab 5: dir {1, -1, 1}
      proj = data[i] - data[i + 1] + data[i + 2];
      if (proj < this.minProj[5]) {
        this.minProj[5] = proj;
        minIndices[5] = i;
      }
      if (proj > this.maxProj[5]) {
        this.maxProj[5] = proj;
        maxIndices[5] = i;
      }

      // Slab 6: dir {1, -1, -1}
      proj = data[i] - data[i + 1] - data[i + 2];
      if (proj < this.minProj[6]) {
        this.minProj[6] = proj;
        minIndices[6] = i;
      }
      if (proj > this.maxProj[6]) {
        this.maxProj[6] = proj;
        maxIndices[6] = i;
      }
    }

    for (let i = 0; i < numPoints; ++i) {
      let index = minIndices[i];
      vec3d.set3(data[index], data[index + 1], data[index + 2], this.minVert[i]);
      index = maxIndices[i];
      vec3d.set3(data[index], data[index + 1], data[index + 2], this.maxVert[i]);
    }
    // Note: Normalization of the extremal projection values can be done here.
    // DiTO-14 only needs the extremal vertices, and the extremal projection
    // values for slab 0-2 (to set the initial AABB).
    // Since unit normals are used for slab 0-2, no normalization is needed.
  }
}

class Orientation {
  b0 = vec3d.createFrom(1, 0, 0); // OBB orientation
  b1 = vec3d.createFrom(0, 1, 0); // OBB orientation
  b2 = vec3d.createFrom(0, 0, 1); // OBB orientation
  quality: number = 0; // evaluation of OBB for orientation
}

function _getQualityValue(len: Vec3d): number {
  return len[0] * len[1] + len[0] * len[2] + len[1] * len[2]; // half box area
}
