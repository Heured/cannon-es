import { Broadphase } from '../collision/Broadphase'
import { Vec3 } from '../math/Vec3'
import type { AABB } from '../collision/AABB'
import type { Body } from '../objects/Body'
import type { World } from '../world/World'

/**
 * Returns the lower bound of an AABB along the given axis.
 */
function GetLower(a: AABB, axis: number): number {
  if (axis === 0) {
    return a.lowerBound.x
  }
  if (axis === 1) {
    return a.lowerBound.y
  }
  return a.lowerBound.z
}

/**
 * Returns the upper bound of an AABB along the given axis.
 */
function GetUpper(a: AABB, axis: number): number {
  if (axis === 0) {
    return a.upperBound.x
  }
  if (axis === 1) {
    return a.upperBound.y
  }
  return a.upperBound.z
}

/**
 * Axis aligned uniform grid broadphase, with sweep and prune inside every cell.
 *
 * The world space is divided into a uniform grid of cells. Every body is
 * inserted into every cell its bounding box overlaps, which acts as a cheap
 * coarse filter. Inside each cell the bodies are then sorted along an axis and
 * swept, exactly like the SAP broadphase, so only pairs that overlap along the
 * axis are tested. Duplicate pairs that appear in more than one cell are
 * removed at the end.
 */
export class GridSAPBroadphase extends Broadphase {
  /**
   * Number of cells along x.
   */
  nx: number

  /**
   * Number of cells along y.
   */
  ny: number

  /**
   * Number of cells along z.
   */
  nz: number

  /**
   * aabbMin
   */
  aabbMin: Vec3

  /**
   * aabbMax
   */
  aabbMax: Vec3

  /**
   * bins
   */
  bins: Body[][]

  /**
   * binLengths
   */
  binLengths: number[]

  /**
   * Axis to sort the bodies along inside every cell.
   * Set to 0 for the x axis, 1 for the y axis and 2 for the z axis.
   * For best performance, pick the axis where bodies are most distributed.
   */
  axisIndex: 0 | 1 | 2

  /**
   * @param aabbMin The minimum corner of the grid.
   * @param aabbMax The maximum corner of the grid.
   * @param nx Number of cells along x.
   * @param ny Number of cells along y.
   * @param nz Number of cells along z.
   */
  constructor(aabbMin = new Vec3(100, 100, 100), aabbMax = new Vec3(-100, -100, -100), nx = 10, ny = 10, nz = 10) {
    super()

    this.nx = nx
    this.ny = ny
    this.nz = nz
    this.aabbMin = aabbMin
    this.aabbMax = aabbMax
    this.axisIndex = 0

    const NBins = this.nx * this.ny * this.nz
    if (NBins <= 0) {
      throw "GridSAPBroadphase: Each dimension's n must be >0"
    }
    this.bins = []
    this.binLengths = [] // Rather than continually resizing arrays (thrashing the memory), just record length and allow them to grow
    this.bins.length = NBins
    this.binLengths.length = NBins
    for (let i = 0; i < NBins; i++) {
      this.bins[i] = []
      this.binLengths[i] = 0
    }
  }

  /**
   * Get all the collision pairs in the physics world.
   */
  collisionPairs(world: World, p1: Body[], p2: Body[]): void {
    const Bodies = world.bodies
    const N = Bodies.length
    const Max = this.aabbMax
    const Min = this.aabbMin
    const Nx = this.nx
    const Ny = this.ny
    const Nz = this.nz
    const AxisIndex = this.axisIndex

    const XStep = Ny * Nz
    const YStep = Nz
    const ZStep = 1

    const XMax = Max.x
    const YMax = Max.y
    const ZMax = Max.z
    const XMin = Min.x
    const YMin = Min.y
    const ZMin = Min.z
    const XMult = Nx / (XMax - XMin)
    const YMult = Ny / (YMax - YMin)
    const ZMult = Nz / (ZMax - ZMin)

    const Bins = this.bins
    const BinLengths = this.binLengths
    const NBins = Bins.length

    // Update AABBs and reset the bins.
    for (let i = 0; i !== N; i++) {
      const Body = Bodies[i]
      if (Body.aabbNeedsUpdate) {
        Body.updateAABB()
      }
    }
    for (let i = 0; i !== NBins; i++) {
      BinLengths[i] = 0
    }

    const Ceil = Math.ceil

    // Put every body into every cell its bounding box overlaps.
    for (let i = 0; i !== N; i++) {
      const Body = Bodies[i]
      const Aabb = Body.aabb

      let XOff0 = ((Aabb.lowerBound.x - XMin) * XMult) | 0
      let YOff0 = ((Aabb.lowerBound.y - YMin) * YMult) | 0
      let ZOff0 = ((Aabb.lowerBound.z - ZMin) * ZMult) | 0
      let XOff1 = Ceil((Aabb.upperBound.x - XMin) * XMult)
      let YOff1 = Ceil((Aabb.upperBound.y - YMin) * YMult)
      let ZOff1 = Ceil((Aabb.upperBound.z - ZMin) * ZMult)

      if (XOff0 < 0) {
        XOff0 = 0
      } else if (XOff0 >= Nx) {
        XOff0 = Nx - 1
      }
      if (YOff0 < 0) {
        YOff0 = 0
      } else if (YOff0 >= Ny) {
        YOff0 = Ny - 1
      }
      if (ZOff0 < 0) {
        ZOff0 = 0
      } else if (ZOff0 >= Nz) {
        ZOff0 = Nz - 1
      }
      if (XOff1 < 0) {
        XOff1 = 0
      } else if (XOff1 >= Nx) {
        XOff1 = Nx - 1
      }
      if (YOff1 < 0) {
        YOff1 = 0
      } else if (YOff1 >= Ny) {
        YOff1 = Ny - 1
      }
      if (ZOff1 < 0) {
        ZOff1 = 0
      } else if (ZOff1 >= Nz) {
        ZOff1 = Nz - 1
      }

      XOff0 *= XStep
      YOff0 *= YStep
      ZOff0 *= ZStep
      XOff1 *= XStep
      YOff1 *= YStep
      ZOff1 *= ZStep

      for (let XOff = XOff0; XOff <= XOff1; XOff += XStep) {
        for (let YOff = YOff0; YOff <= YOff1; YOff += YStep) {
          for (let ZOff = ZOff0; ZOff <= ZOff1; ZOff += ZStep) {
            const Index = XOff + YOff + ZOff
            Bins[Index][BinLengths[Index]++] = Body
          }
        }
      }
    }

    // Sweep and prune inside every cell.
    for (let i = 0; i !== NBins; i++) {
      const BinLength = BinLengths[i]
      if (BinLength > 1) {
        const Bin = Bins[i]

        // Only sort the bodies that were actually added to this cell.
        Bin.length = BinLength

        // Sort along the chosen axis, by the lower bound of the bounding box.
        Bin.sort((A, B) => GetLower(A.aabb, AxisIndex) - GetLower(B.aabb, AxisIndex))

        for (let Xi = 0; Xi !== BinLength; Xi++) {
          const BodyA = Bin[Xi]
          const MaxBound = GetUpper(BodyA.aabb, AxisIndex)

          for (let Yi = Xi + 1; Yi !== BinLength; Yi++) {
            const BodyB = Bin[Yi]
            if (GetLower(BodyB.aabb, AxisIndex) > MaxBound) {
              break
            }

            if (!this.needBroadphaseCollision(BodyA, BodyB)) {
              continue
            }

            this.intersectionTest(BodyA, BodyB, p1, p2)
          }
        }
      }
    }

    // A body that spans several cells produces the same pair in each of
    // them, so remove the duplicates before returning.
    this.makePairsUnique(p1, p2)
  }

  /**
   * Returns all the bodies within an AABB.
   * @param result An array to store resulting bodies in.
   */
  aabbQuery(world: World, aabb: AABB, result: Body[] = []): Body[] {
    for (let i = 0; i < world.bodies.length; i++) {
      const Body = world.bodies[i]

      if (Body.aabbNeedsUpdate) {
        Body.updateAABB()
      }

      if (Body.aabb.overlaps(aabb)) {
        result.push(Body)
      }
    }

    return result
  }
}
