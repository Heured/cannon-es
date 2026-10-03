import { Quaternion } from '../math/Quaternion'
import { Pool } from '../utils/Pool'

/**
 * Vec3Pool
 */
export class QuatPool extends Pool {
  type = Quaternion

  /**
   * Construct a vector
   */
  constructObject(): Quaternion {
    return new Quaternion()
  }
}
