import { Broadphase } from './Broadphase'
import type { AABB } from './AABB'
import type { Body } from '../objects/Body'
import type { World } from '../world/World'

/**
 * Multi Box Pruning broadphase.
 *
 * The bodies are sorted along the x axis by the lower bound of their bounding
 * box, then swept to collect candidate pairs that overlap on x. Each candidate
 * pair is filtered against the y and z axes, so only pairs whose bounding boxes
 * overlap on all three axes are reported. This reduces the O(N^2) work of the
 * naive broadphase to the number of pairs that actually overlap on x.
 */
export class MultiBoxPruningBroadphase extends Broadphase {
  /**
   * The bodies of the world, sorted along the x axis. Reused across steps to
   * avoid allocations.
   */
  private AxisList: Body[]

  constructor() {
    super()

    this.AxisList = []
  }

  /**
   * Get all the collision pairs in the physics world.
   */
  collisionPairs(world: World, p1: Body[], p2: Body[]): void {
    const Bodies = world.bodies
    const N = Bodies.length

    // Update AABBs and copy the bodies into the sorted list.
    const AxisList = this.AxisList
    AxisList.length = 0
    for (let i = 0; i !== N; i++) {
      const Body = Bodies[i]
      if (Body.aabbNeedsUpdate) {
        Body.updateAABB()
      }
      AxisList.push(Body)
    }

    // Sort along the x axis by the lower bound of the bounding box.
    AxisList.sort((a, b) => a.aabb.lowerBound.x - b.aabb.lowerBound.x)

    let j

    // Sweep along x, testing the y and z axes for each candidate pair.
    for (let i = 0, ni = N - 1; i < ni; i++) {
      const BodyA = AxisList[i]
      const MaxX = BodyA.aabb.upperBound.x

      for (j = i + 1; j < N; j++) {
        const BodyB = AxisList[j]
        if (BodyB.aabb.lowerBound.x > MaxX) {
          break
        }

        if (!MultiBoxPruningBroadphase.OverlapsOnY(BodyA.aabb, BodyB.aabb)) {
          continue
        }

        if (!MultiBoxPruningBroadphase.OverlapsOnZ(BodyA.aabb, BodyB.aabb)) {
          continue
        }

        if (!this.needBroadphaseCollision(BodyA, BodyB)) {
          continue
        }

        this.intersectionTest(BodyA, BodyB, p1, p2)
      }
    }
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

  /**
   * Checks if two AABBs overlap along the y axis.
   */
  private static OverlapsOnY(a: AABB, b: AABB): boolean {
    return a.lowerBound.y <= b.upperBound.y && b.lowerBound.y <= a.upperBound.y
  }

  /**
   * Checks if two AABBs overlap along the z axis.
   */
  private static OverlapsOnZ(a: AABB, b: AABB): boolean {
    return a.lowerBound.z <= b.upperBound.z && b.lowerBound.z <= a.upperBound.z
  }
}
