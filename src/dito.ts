
export interface Obb {
  center: Float64Array;
  halfSize: Float32Array;
  quaternion: Float32Array;
}

export interface Attribute {
  /** Data uses the data type of the vertex attribute */
  data: number[] | Float64Array | Float32Array;
  /** Components per vertex */
  size: number;

  /** Index into data array i.e. not a byte offset  */
  offsetIdx: number;
  /** Stride across data array i.e. not a byte stride */
  strideIdx: number;
}

namespace glMatrix {
  const tmpV3 = vec3d.create();

  export namespace vec3d {
    export function create(): Float64Array { return new Float64Array(3); }
    export function createFrom(x: number, y: number, z: number): Float64Array {
      return set3(x, y, z, new Float64Array(3));
    }
    export function add(a: Float64Array, b: Float64Array, d?: Float64Array): Float64Array {
      if (!d)
        d = a;

      d[0] = a[0] + b[0];
      d[1] = a[1] + b[1];
      d[2] = a[2] + b[2];
      return d;
    }
    export function set(from: Float64Array, d: Float64Array) {
      d[0] = from[0];
      d[1] = from[1];
      d[2] = from[2];
    }
    export function set3(x: number, y: number, z: number, d: Float64Array): Float64Array {
      d[0] = x;
      d[1] = y;
      d[2] = z;
      return d;
    }
    export function subtract(a: Float64Array, b: Float64Array, d?: Float64Array): Float64Array {
      if (!d)
        d = a;

      d[0] = a[0] - b[0];
      d[1] = a[1] - b[1];
      d[2] = a[2] - b[2];
      return d;
    }
    export function scale(a: Float64Array, s: number, d?: Float64Array): Float64Array {
      if (!d)
        d = a;

      d[0] = a[0] * s;
      d[1] = a[1] * s;
      d[2] = a[2] * s;
      return d;
    }
    export function length2(a: Float64Array): number {
      return a[0] * a[0] + a[1] * a[1] + a[2] * a[2];
    }
    export function length(a: Float64Array): number {
      return Math.sqrt(length2(a));
    }
    export function dist2(a: Float64Array, b: Float64Array): number {
      subtract(a, b, tmpV3);
      return length2(tmpV3);
    }
    export function normalize(a: Float64Array, d?: Float64Array): Float64Array {
      return scale(a, 1 / length(a), d);
    }
    export function cross(a: Float64Array, b: Float64Array, d?: Float64Array): Float64Array {
      if (!d)
        d = a;

      const x1 = a[0], y1 = a[1], z1 = a[2];
      const x2 = b[0], y2 = b[1], z2 = b[2];
      d[0] = y1 * z2 - z1 * y2;
      d[1] = z1 * x2 - x1 * z2;
      d[2] = x1 * y2 - y1 * x2;
      return d;
    }
    export function dot(a: Float64Array, b: Float64Array): number {
      return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }
  }
  export namespace vec2d {
    export function create(): Float64Array { return new Float64Array(2); }
  }
  export namespace quat4 {
    export function identity(a: Float32Array) {
      a[0] = 0;
      a[1] = 0;
      a[2] = 0;
      a[3] = 1;
    }
    export function fromRotationMatrix(m: Float64Array, d: Float32Array): Float32Array {

      // Algorithm in Ken Shoemake's article in 1987 SIGGRAPH course notes
      // article "Quaternion Calculus and Fast Animation".

      const fTrace = m[0] + m[4] + m[8];
      if (fTrace > 0.0) {
        // |w| > 1/2, may as well choose w > 1/2
        let fRoot = Math.sqrt(fTrace + 1.0); // 2w
        d[3] = 0.5 * fRoot;
        fRoot = 0.5 / fRoot; // 1/(4w)
        d[0] = (m[7] - m[5]) * fRoot;
        d[1] = (m[2] - m[6]) * fRoot;
        d[2] = (m[3] - m[1]) * fRoot;
      } else {
        // |w| <= 1/2
        const s_iNext = [1, 2, 0];
        var i = 0;
        if (m[4] > m[0])
          i = 1;
        if (m[8] > m[i * 3 + i])
          i = 2;
        var j = s_iNext[i];
        var k = s_iNext[j];

        let fRoot = Math.sqrt(m[i * 3 + i] - m[j * 3 + j] - m[k * 3 + k] + 1.0);
        d[i] = 0.5 * fRoot;
        fRoot = 0.5 / fRoot;
        d[3] = (m[k * 3 + j] - m[j * 3 + k]) * fRoot;
        d[j] = (m[j * 3 + i] + m[i * 3 + j]) * fRoot;
        d[k] = (m[k * 3 + i] + m[i * 3 + k]) * fRoot;
      }

      return d;
    }
  }
  export namespace mat3d {
    export function create(): Float64Array { return new Float64Array(9); }
  }
}

const epsilon: number = 0.000001;
const alMid = glMatrix.vec3d.create();
const alLen = glMatrix.vec3d.create();

// Derived from the C++ sample implementation of http://www.idt.mdh.se/~tla/publ/FastOBBs.pdf
export function computeOBB(positions: Attribute, obb: Obb) {

  const { data, strideIdx } = positions;
  const count = data.length / strideIdx;
  if (count <= 0) {
    return;
  }

  // Select seven extremal points along predefined slab directions
  const extremals: ExtremalPoints = new ExtremalPoints(positions);

  // Compute size of AABB (max and min projections of vertices are already
  // computed as slabs 0-2)
  glMatrix.vec3d.add(extremals.minProj, extremals.maxProj, alMid);
  glMatrix.vec3d.scale(alMid, 0.5);

  glMatrix.vec3d.subtract(extremals.maxProj, extremals.minProj, alLen);

  const alVal = _getQualityValue(alLen);
  const best = new Orientation();
  best.quality = alVal;

  if (count < 14) {
    positions = { data: new Float64Array(extremals.buffer, 14 * 8, 14 * 3), size: 3, offsetIdx: 0, strideIdx: 3 };
  }

  // Find best OBB axes based on the constructed base triangle
  const p0 = glMatrix.vec3d.create(); // Vertices of the large base triangle
  const p1 = glMatrix.vec3d.create(); // Vertices of the large base triangle
  const p2 = glMatrix.vec3d.create(); // Vertices of the large base triangle
  const e0 = glMatrix.vec3d.create(); // Edge vectors of the large base triangle
  const e1 = glMatrix.vec3d.create(); // Edge vectors of the large base triangle
  const e2 = glMatrix.vec3d.create(); // Edge vectors of the large base triangle
  const n = glMatrix.vec3d.create(); // Unit normal of the large base triangle

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
  const bLen = glMatrix.vec3d.create();
  glMatrix.vec3d.subtract(bMax, bMin, bLen);
  best.quality = _getQualityValue(bLen);

  // Check if the OBB extent is still smaller than the intial AABB
  if (best.quality < alVal) {
    _finalizeOBB(best.b0, best.b1, best.b2, bMin, bMax, bLen, obb); // if so, assign all OBB params
  } else {
    _finalizeAxisAlignedOBB(alMid, alLen, obb); // otherwise, assign all OBB params using the intial AABB
  }
}

function _findBestObbAxesFromBaseTriangle(extremals: ExtremalPoints, positions: Attribute, n: Float64Array,
  p0: Float64Array, p1: Float64Array, p2: Float64Array,
  e0: Float64Array, e1: Float64Array, e2: Float64Array,
  best: Orientation, obb: Obb) {

  _findFurthestPointPair(extremals, p0, p1);

  // Degenerate case 1:
  // If the furthest points are very close, return OBB aligned with the initial AABB
  if (glMatrix.vec3d.dist2(p0, p1) < epsilon) {
    return 1;
  }

  // Compute edge vector of the line segment p0, p1
  glMatrix.vec3d.subtract(p0, p1, e0);
  glMatrix.vec3d.normalize(e0);

  // Find a third point furthest away from line given by p0, e0 to define the large base triangle
  const dist2 = _findFurthestPointFromInfiniteEdge(positions, p0, e0, p2);

  // Degenerate case 2:
  // If the third point is located very close to the line, return an OBB aligned with the line
  if (dist2 < epsilon) {
    return 2;
  }

  // Compute the two remaining edge vectors and the normal vector of the base triangle
  glMatrix.vec3d.subtract(p1, p2, e1);
  glMatrix.vec3d.normalize(e1);
  glMatrix.vec3d.subtract(p2, p0, e2);
  glMatrix.vec3d.normalize(e2);
  glMatrix.vec3d.cross(e1, e0, n);
  glMatrix.vec3d.normalize(n);

  _findBestObbAxesFromTriangleNormalAndEdgeVectors(positions, n, e0, e1, e2, best);
  return 0; // success
}

const q0 = glMatrix.vec3d.create();
const q1 = glMatrix.vec3d.create();
const f0 = glMatrix.vec3d.create(); // Edge vectors towards q0/1
const f1 = glMatrix.vec3d.create(); // Edge vectors towards q0/1
const f2 = glMatrix.vec3d.create(); // Edge vectors towards q0/1
const n0 = glMatrix.vec3d.create(); // Unit normals of tetra tris
const n1 = glMatrix.vec3d.create(); // Unit normals of tetra tris
const n2 = glMatrix.vec3d.create(); // Unit normals of tetra tris

function _findImprovedObbAxesFromUpperAndLowerTetrasOfBaseTriangle(positions: Attribute, n: Float64Array,
  p0: Float64Array, p1: Float64Array, p2: Float64Array,
  e0: Float64Array, e1: Float64Array, e2: Float64Array,
  best: Orientation, obb: Obb) {

  // Find furthest points above and below the plane of the base triangle for tetra constructions
  _findUpperLowerTetraPoints(positions, n, p0, p1, p2, q0, q1);

  // For each valid point found, search for the best OBB axes based on the 3 arising triangles
  if (q0[0] !== undefined) {

    glMatrix.vec3d.subtract(q0, p0, f0);
    glMatrix.vec3d.normalize(f0);
    glMatrix.vec3d.subtract(q0, p1, f1);
    glMatrix.vec3d.normalize(f1);
    glMatrix.vec3d.subtract(q0, p2, f2);
    glMatrix.vec3d.normalize(f2);

    glMatrix.vec3d.cross(f1, e0, n0);
    glMatrix.vec3d.normalize(n0);
    glMatrix.vec3d.cross(f2, e1, n1);
    glMatrix.vec3d.normalize(n1);
    glMatrix.vec3d.cross(f0, e2, n2);
    glMatrix.vec3d.normalize(n2);

    _findBestObbAxesFromTriangleNormalAndEdgeVectors(positions, n0, e0, f1, f0, best);
    _findBestObbAxesFromTriangleNormalAndEdgeVectors(positions, n1, e1, f2, f1, best);
    _findBestObbAxesFromTriangleNormalAndEdgeVectors(positions, n2, e2, f0, f2, best);
  }
  if (q1[0] !== undefined) {

    glMatrix.vec3d.subtract(q1, p0, f0);
    glMatrix.vec3d.normalize(f0);
    glMatrix.vec3d.subtract(q1, p1, f1);
    glMatrix.vec3d.normalize(f1);
    glMatrix.vec3d.subtract(q1, p2, f2);
    glMatrix.vec3d.normalize(f2);

    glMatrix.vec3d.cross(f1, e0, n0);
    glMatrix.vec3d.normalize(n0);
    glMatrix.vec3d.cross(f2, e1, n1);
    glMatrix.vec3d.normalize(n1);
    glMatrix.vec3d.cross(f0, e2, n2);
    glMatrix.vec3d.normalize(n2);

    _findBestObbAxesFromTriangleNormalAndEdgeVectors(positions, n0, e0, f1, f0, best);
    _findBestObbAxesFromTriangleNormalAndEdgeVectors(positions, n1, e1, f2, f1, best);
    _findBestObbAxesFromTriangleNormalAndEdgeVectors(positions, n2, e2, f0, f2, best);
  }
}

function _findFurthestPointPair(extremals: ExtremalPoints, p0: Float64Array, p1: Float64Array) {

  let maxDist2 = glMatrix.vec3d.dist2(extremals.maxVert[0], extremals.minVert[0]);
  let index: number = 0;

  for (let i = 1; i < 7; ++i) {
    const dist2 = glMatrix.vec3d.dist2(extremals.maxVert[i], extremals.minVert[i]);
    if (dist2 > maxDist2) {
      maxDist2 = dist2;
      index = i;
    }
  }
  glMatrix.vec3d.set(extremals.minVert[index], p0);
  glMatrix.vec3d.set(extremals.maxVert[index], p1);
}

const u0 = glMatrix.vec3d.create();

function _findFurthestPointFromInfiniteEdge(positions: Attribute, p0: Float64Array, e0: Float64Array,
  p: Float64Array): number {

  const { data, offsetIdx, strideIdx } = positions;

  let maxDist2 = Number.NEGATIVE_INFINITY;
  let maxIndex: number = 0;

  for (let i = offsetIdx; i < data.length; i += strideIdx) {
    // inlined _dist2PointInfiniteEdge
    glMatrix.vec3d.set3(data[i] - p0[0], data[i + 1] - p0[1], data[i + 2] - p0[2], u0);
    const t = e0[0] * u0[0] + e0[1] * u0[1] + e0[2] * u0[2];
    const sqLen_e0 = e0[0] * e0[0] + e0[1] * e0[1] + e0[2] * e0[2];
    const sqLen_u0 = u0[0] * u0[0] + u0[1] * u0[1] + u0[2] * u0[2];
    const dist2 = sqLen_u0 - (t * t) / sqLen_e0;

    if (dist2 > maxDist2) {
      maxDist2 = dist2;
      maxIndex = i;
    }
  }

  glMatrix.vec3d.set3(data[maxIndex], data[maxIndex + 1], data[maxIndex + 2], p);
  return maxDist2;
}

const minmax = glMatrix.vec2d.create();

function _findUpperLowerTetraPoints(positions: Attribute, n: Float64Array,
  p0: Float64Array, p1: Float64Array, p2: Float64Array,
  q0: Float64Array, q1: Float64Array) {

  _findExtremalPoints_OneDir(positions, n, minmax, q1, q0);
  const triProj = glMatrix.vec3d.dot(p0, n);

  if (minmax[1] - epsilon <= triProj) {
    q0[0] = undefined; // invalidate
  }
  if (minmax[0] + epsilon >= triProj) {
    q1[0] = undefined; // invalidate
  }
}

const m0 = glMatrix.vec3d.create();
const m1 = glMatrix.vec3d.create();
const m2 = glMatrix.vec3d.create();
const dmax = glMatrix.vec3d.create();
const dmin = glMatrix.vec3d.create();
const dlen = glMatrix.vec3d.create();

function _findBestObbAxesFromTriangleNormalAndEdgeVectors(positions: Attribute, n: Float64Array,
  e0: Float64Array, e1: Float64Array, e2: Float64Array, best: Orientation) {

  if (glMatrix.vec3d.length2(n) < epsilon) {
    return;
  }

  glMatrix.vec3d.cross(e0, n, m0);
  glMatrix.vec3d.cross(e1, n, m1);
  glMatrix.vec3d.cross(e2, n, m2);

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
      glMatrix.vec3d.set(edges[i], best.b0);
      glMatrix.vec3d.set(n, best.b1);
      glMatrix.vec3d.set(ems[i], best.b2);
      best.quality = quality;
    }
  }
}

const point = glMatrix.vec3d.create();

function _findExtremalProjs_OneDir(positions: Attribute, n: Float64Array, minmax: Float64Array) {

  const { data, offsetIdx, strideIdx } = positions;

  minmax[0] = Number.POSITIVE_INFINITY;
  minmax[1] = Number.NEGATIVE_INFINITY;

  for (let i = offsetIdx; i < data.length; i += strideIdx) {
    const proj = data[i] * n[0] + data[i + 1] * n[1] + data[i + 2] * n[2]; // opt: inline dot product
    minmax[0] = Math.min(minmax[0], proj);
    minmax[1] = Math.max(minmax[1], proj);
  }
}

function _findExtremalPoints_OneDir(positions: Attribute, n: Float64Array, minmax: Float64Array,
  minVert: Float64Array, maxVert: Float64Array) {

  const { data, offsetIdx, strideIdx } = positions;
  glMatrix.vec3d.set3(data[offsetIdx], data[offsetIdx + 1], data[offsetIdx + 2], minVert);
  glMatrix.vec3d.set(minVert, maxVert);

  minmax[0] = glMatrix.vec3d.dot(point, n);
  minmax[1] = minmax[0];

  for (let i = offsetIdx + strideIdx; i < data.length; i += strideIdx) {
    const proj = data[i] * n[0] + data[i + 1] * n[1] + data[i + 2] * n[2];

    if (proj < minmax[0]) {
      minmax[0] = proj;
      glMatrix.vec3d.set3(data[i], data[i + 1], data[i + 2], minVert);
    }
    if (proj > minmax[1]) {
      minmax[1] = proj;
      glMatrix.vec3d.set3(data[i], data[i + 1], data[i + 2], maxVert);
    }
  }
}

function _finalizeAxisAlignedOBB(mid: Float64Array, len: Float64Array, obb: Obb) {

  glMatrix.vec3d.set(mid, obb.center);
  obb.halfSize[0] = len[0] * 0.5;
  obb.halfSize[1] = len[1] * 0.5;
  obb.halfSize[2] = len[2] * 0.5;
  glMatrix.quat4.identity(obb.quaternion);
}

const r = glMatrix.vec3d.create();
const v = glMatrix.vec3d.create();
const w = glMatrix.vec3d.create();
const bMin = glMatrix.vec3d.create();
const bMax = glMatrix.vec3d.create();
const bLen = glMatrix.vec3d.create();

// This function is only called if the construction of the large base triangle fails
function _finalizeLineAlignedOBB(positions: Attribute, u: Float64Array, obb: Obb) {

  // Given u, build any orthonormal base u, v, w
  // Make sure r is not equal to u
  glMatrix.vec3d.set(u, r);
  if (Math.abs(u[0]) > Math.abs(u[1]) && Math.abs(u[0]) > Math.abs(u[2])) {
    r[0] = 0;
  }
  else if (Math.abs(u[1]) > Math.abs(u[2])) {
    r[1] = 0;
  }
  else {
    r[2] = 0;
  }

  if (glMatrix.vec3d.length2(r) < epsilon) {
    r[0] = r[1] = r[2] = 1;
  }

  glMatrix.vec3d.cross(u, r, v);
  glMatrix.vec3d.normalize(v);
  glMatrix.vec3d.cross(u, v, w);
  glMatrix.vec3d.normalize(w);

  // compute the true obb dimensions by iterating over all vertices
  _computeObbDimensions(positions, u, v, w, bMin, bMax);
  glMatrix.vec3d.subtract(bMax, bMin, bLen);
  _finalizeOBB(u, v, w, bMin, bMax, bLen, obb);
}

function _computeObbDimensions(positions: Attribute, v0: Float64Array, v1: Float64Array, v2: Float64Array,
  min: Float64Array, max: Float64Array) {

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

const tmp = glMatrix.vec3d.create();
const rot = glMatrix.mat3d.create();
const q = glMatrix.vec3d.create();

function _finalizeOBB(v0: Float64Array, v1: Float64Array, v2: Float64Array,
  min: Float64Array, max: Float64Array, len: Float64Array, obb: Obb) {

  rot[0] = v0[0]; rot[3] = v0[1]; rot[6] = v0[2];
  rot[1] = v1[0]; rot[4] = v1[1]; rot[7] = v1[2];
  rot[2] = v2[0]; rot[5] = v2[1]; rot[8] = v2[2];
  glMatrix.quat4.fromRotationMatrix(rot, obb.quaternion);

  // midpoint expressed in the OBB's own coordinate system
  glMatrix.vec3d.add(min, max, q);
  glMatrix.vec3d.scale(q, 0.5);

  // Compute midpoint expressed in the standard base
  glMatrix.vec3d.scale(v0, q[0], obb.center);
  glMatrix.vec3d.scale(v1, q[1], tmp);
  glMatrix.vec3d.add(obb.center, tmp);
  glMatrix.vec3d.scale(v2, q[2], tmp);
  glMatrix.vec3d.add(obb.center, tmp);

  obb.halfSize[0] = len[0] * 0.5;
  obb.halfSize[1] = len[1] * 0.5;
  obb.halfSize[2] = len[2] * 0.5;
}

const numPoints = 7;

class ExtremalPoints {
  buffer: ArrayBuffer;

  minProj: Float64Array;
  maxProj: Float64Array;
  minVert: Float64Array[] = new Array(numPoints);
  maxVert: Float64Array[] = new Array(numPoints);

  constructor(positions: Attribute) {
    // setup storage
    const bufferSize = numPoints * (8 + 8 + 24 + 24);
    this.buffer = new ArrayBuffer(bufferSize);

    let offset: number = 0;
    this.minProj = new Float64Array(this.buffer, offset, numPoints);
    offset += numPoints * 8;

    this.maxProj = new Float64Array(this.buffer, offset, numPoints);
    offset += numPoints * 8;

    for (let i = 0; i < numPoints; ++i) {
      this.minVert[i] = new Float64Array(this.buffer, offset, 3);
      offset += 24;
    }
    for (let i = 0; i < numPoints; ++i) {
      this.maxVert[i] = new Float64Array(this.buffer, offset, 3);
      offset += 24;
    }

    // init storage
    for (let i = 0; i < numPoints; ++i) {
      this.minProj[i] = Number.POSITIVE_INFINITY;
      this.maxProj[i] = Number.NEGATIVE_INFINITY;
    }
    const minIndices: number[] = new Array(numPoints);
    const maxIndices: number[] = new Array(numPoints);

    const { data, offsetIdx, strideIdx } = positions;

    // find extremal points
    for (let i = offsetIdx; i < data.length; i += strideIdx) {
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
      glMatrix.vec3d.set3(data[index], data[index + 1], data[index + 2], this.minVert[i]);
      index = maxIndices[i];
      glMatrix.vec3d.set3(data[index], data[index + 1], data[index + 2], this.maxVert[i]);
    }
    // Note: Normalization of the extremal projection values can be done here.
    // DiTO-14 only needs the extremal vertices, and the extremal projection
    // values for slab 0-2 (to set the initial AABB).
    // Since unit normals are used for slab 0-2, no normalization is needed.
  }
}

class Orientation {
  b0 = glMatrix.vec3d.createFrom(1, 0, 0); // OBB orientation
  b1 = glMatrix.vec3d.createFrom(0, 1, 0); // OBB orientation
  b2 = glMatrix.vec3d.createFrom(0, 0, 1); // OBB orientation
  quality: number = 0; // evaluation of OBB for orientation
}

function _getQualityValue(len: Float64Array): number {
  return len[0] * len[1] + len[0] * len[2] + len[1] * len[2]; // half box area
}
