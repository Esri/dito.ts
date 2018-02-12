describe('dito', function () {
  const jaspi3xUrl = 'base/test/mocks/jsapi3x.js';
  describe('obb construction', function () {
    var v = [-6, -5, -4,  3, 2, 1,  -1, -1, 1,  1, -1, 1,
             1, 1, 1,  -1, 1, 1,  -1, -1, -1,  1, -1, -1,
             1, 1, -1,  -1, 1, -1,  -0.5, -0.5, 1,  1, 0.5, -0.5,
             0.5, 0.5, -1,  -1, -0.5, 0.5];
    const obb = orientedBoundingBox.compute({data:v, size: 3, offsetIdx: 0, strideIdx:3});
    it('center', function () {
      expect(obb.center).toEqual([-1.5, -1.8, -0.9]);
    });
    it('halfSize', function () {
      expect(obb.center).toEqual([6.09566593170166, 1.4084056615829468, 1.4385133981704712]);
    });
    it('quaternion', function () {
      expect(obb.center).toEqual([0.8904075026512146, 0.41704457998275757, 0.1651262491941452, -0.0773409977555275]);
    });
  });
});
