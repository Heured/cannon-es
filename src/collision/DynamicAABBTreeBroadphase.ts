import { Broadphase } from '../collision/Broadphase'
import { AABB } from '../collision/AABB'
import type { Body } from '../objects/Body'
import type { World } from '../world/World'

/**
 * Computes the surface area of an AABB.
 */
function SurfaceArea(a: AABB): number {
  const Dx = a.upperBound.x - a.lowerBound.x
  const Dy = a.upperBound.y - a.lowerBound.y
  const Dz = a.upperBound.z - a.lowerBound.z
  return 2 * (Dx * Dy + Dy * Dz + Dz * Dx)
}

/**
 * Computes the surface area of the union of two AABBs, without allocating.
 */
function CombinedSurfaceArea(a: AABB, b: AABB): number {
  const Dx = Math.max(a.upperBound.x, b.upperBound.x) - Math.min(a.lowerBound.x, b.lowerBound.x)
  const Dy = Math.max(a.upperBound.y, b.upperBound.y) - Math.min(a.lowerBound.y, b.lowerBound.y)
  const Dz = Math.max(a.upperBound.z, b.upperBound.z) - Math.min(a.lowerBound.z, b.lowerBound.z)
  return 2 * (Dx * Dy + Dy * Dz + Dz * Dx)
}

/**
 * Stores the union of two AABBs into the out AABB.
 */
function SetUnion(a: AABB, b: AABB, out: AABB): void {
  out.lowerBound.x = Math.min(a.lowerBound.x, b.lowerBound.x)
  out.lowerBound.y = Math.min(a.lowerBound.y, b.lowerBound.y)
  out.lowerBound.z = Math.min(a.lowerBound.z, b.lowerBound.z)
  out.upperBound.x = Math.max(a.upperBound.x, b.upperBound.x)
  out.upperBound.y = Math.max(a.upperBound.y, b.upperBound.y)
  out.upperBound.z = Math.max(a.upperBound.z, b.upperBound.z)
}

/**
 * A node of the Dynamic AABB Tree.
 *
 * Leaf nodes hold a single body, while internal nodes hold the AABB of their
 * subtree. The AABB stored on every node is a "fat" AABB, inflated by a small
 * margin, so that slowly moving bodies do not need to be reinserted every step.
 */
class DynamicAABBNode {
  /**
   * The (fat) axis aligned bounding box of this node.
   */
  Aabb: AABB

  /**
   * The parent node, or null if this node is the root.
   */
  Parent: DynamicAABBNode | null

  /**
   * The first child, or null if this node is a leaf.
   */
  Child1: DynamicAABBNode | null

  /**
   * The second child, or null if this node is a leaf.
   */
  Child2: DynamicAABBNode | null

  /**
   * The body this leaf represents, or null for internal nodes.
   */
  Body: Body | null

  /**
   * The height of the subtree rooted at this node. Leaf nodes have height 0.
   */
  Height: number

  /**
   * A unique identifier used to de-duplicate collision pairs.
   */
  Id: number

  constructor() {
    this.Aabb = new AABB()
    this.Parent = null
    this.Child1 = null
    this.Child2 = null
    this.Body = null
    this.Height = 0
    this.Id = 0
  }

  /**
   * Returns true if this node is a leaf node.
   */
  IsLeaf(): boolean {
    return this.Child1 === null
  }
}

/**
 * Dynamic AABB Tree broadphase.
 *
 * Bodies are inserted into a bounding volume hierarchy of axis aligned bounding
 * boxes. The tree is rebuilt incrementally as bodies move, and overlapping
 * leaves are reported as collision candidates. This makes it well suited for
 * worlds with many dynamic bodies and few static ones.
 */
export class DynamicAABBTreeBroadphase extends Broadphase {
  /**
   * The root of the tree.
   */
  Root: DynamicAABBNode | null

  /**
   * The margin added to the AABB of every body. A larger margin keeps bodies
   * in the tree for longer before they are reinserted, at the cost of more
   * candidate pairs.
   */
  FatAABBMargin: number

  /**
   * The bodies currently inserted in the tree.
   */
  Bodies: Body[]

  /**
   * Maps each body to its leaf node.
   */
  private BodyToLeaf: Map<Body, DynamicAABBNode>

  /**
   * A pool of reusable tree nodes.
   */
  private FreeList: DynamicAABBNode[]

  /**
   * A counter used to give every leaf node a unique identifier.
   */
  private NextId: number

  /**
   * A reusable stack for the tree queries.
   */
  private Stack: DynamicAABBNode[]

  private AddBodyHandler: (event: { body: Body }) => void
  private RemoveBodyHandler: (event: { body: Body }) => void

  constructor(world?: World) {
    super()

    this.Root = null
    this.FatAABBMargin = 0.1
    this.Bodies = []
    this.BodyToLeaf = new Map()
    this.FreeList = []
    this.NextId = 0
    this.Stack = []

    this.AddBodyHandler = (event: { body: Body }) => {
      this.InsertBody(event.body)
    }

    this.RemoveBodyHandler = (event: { body: Body }) => {
      this.RemoveBody(event.body)
    }

    if (world) {
      this.setWorld(world)
    }
  }

  /**
   * Change the world. The tree is cleared and rebuilt from the new world.
   */
  setWorld(world: World): void {
    this.Clear()

    for (let i = 0; i < world.bodies.length; i++) {
      this.InsertBody(world.bodies[i])
    }

    const oldWorld = this.world
    if (oldWorld) {
      oldWorld.removeEventListener('addBody', this.AddBodyHandler)
      oldWorld.removeEventListener('removeBody', this.RemoveBodyHandler)
    }

    world.addEventListener('addBody', this.AddBodyHandler)
    world.addEventListener('removeBody', this.RemoveBodyHandler)

    this.world = world
    this.dirty = true
  }

  /**
   * Remove all bodies from the tree.
   */
  Clear(): void {
    this.Root = null
    this.Bodies.length = 0
    this.BodyToLeaf.clear()
    this.FreeList.length = 0
    this.NextId = 0
  }

  /**
   * Insert a body into the tree.
   */
  InsertBody(body: Body): void {
    if (this.BodyToLeaf.has(body)) {
      return
    }

    if (body.aabbNeedsUpdate) {
      body.updateAABB()
    }

    const Leaf = this.AllocateNode(body)
    this.SetFatAabb(body, Leaf.Aabb)
    this.InsertLeaf(Leaf)
    Leaf.Id = this.NextId++

    this.BodyToLeaf.set(body, Leaf)
    this.Bodies.push(body)
  }

  /**
   * Remove a body from the tree.
   */
  RemoveBody(body: Body): void {
    const Leaf = this.BodyToLeaf.get(body)
    if (Leaf === undefined) {
      return
    }

    this.RemoveLeaf(Leaf)
    this.FreeList.push(Leaf)

    this.BodyToLeaf.delete(body)
    const Index = this.Bodies.indexOf(body)
    if (Index !== -1) {
      this.Bodies.splice(Index, 1)
    }
  }

  /**
   * Collect all collision pairs.
   */
  collisionPairs(world: World, p1: Body[], p2: Body[]): void {
    this.UpdateTree()

    const Bodies = this.Bodies
    for (let i = 0; i !== Bodies.length; i++) {
      const QueryLeaf = this.BodyToLeaf.get(Bodies[i])!

      this.QueryTree(QueryLeaf.Aabb, (FoundLeaf) => {
        if (FoundLeaf.Id <= QueryLeaf.Id) {
          return
        }

        const BodyA = QueryLeaf.Body!
        const BodyB = FoundLeaf.Body!
        if (!this.needBroadphaseCollision(BodyA, BodyB)) {
          return
        }

        this.intersectionTest(BodyA, BodyB, p1, p2)
      })
    }
  }

  /**
   * Returns all the bodies within an AABB.
   * @param result An array to store resulting bodies in.
   */
  aabbQuery(world: World, aabb: AABB, result: Body[] = []): Body[] {
    this.UpdateTree()

    this.QueryTree(aabb, (Leaf) => {
      const Body = Leaf.Body!
      if (Body.aabb.overlaps(aabb)) {
        result.push(Body)
      }
    })

    return result
  }

  /**
   * Reinserts every body whose AABB has left its fat AABB.
   */
  private UpdateTree(): void {
    const Bodies = this.Bodies
    for (let i = 0; i !== Bodies.length; i++) {
      const Body = Bodies[i]
      if (Body.aabbNeedsUpdate) {
        Body.updateAABB()

        const Leaf = this.BodyToLeaf.get(Body)
        if (Leaf !== undefined && !Leaf.Aabb.contains(Body.aabb)) {
          this.RemoveLeaf(Leaf)
          this.SetFatAabb(Body, Leaf.Aabb)
          this.InsertLeaf(Leaf)
        }
      }
    }
  }

  /**
   * Queries the tree for every leaf whose fat AABB overlaps the given AABB.
   */
  private QueryTree(aabb: AABB, callback: (leaf: DynamicAABBNode) => void): void {
    const Stack = this.Stack
    Stack.length = 0

    if (this.Root === null) {
      return
    }

    Stack.push(this.Root)
    while (Stack.length > 0) {
      const Node = Stack.pop()!

      if (Node.IsLeaf()) {
        if (Node.Aabb.overlaps(aabb)) {
          callback(Node)
        }
      } else {
        if (Node.Aabb.overlaps(aabb)) {
          Stack.push(Node.Child1!)
          Stack.push(Node.Child2!)
        }
      }
    }
  }

  /**
   * Inflates the body AABB by the fat margin and stores it in out.
   */
  private SetFatAabb(body: Body, out: AABB): void {
    out.copy(body.aabb)

    const Margin = this.FatAABBMargin + 0.05 * body.boundingRadius
    out.lowerBound.x -= Margin
    out.lowerBound.y -= Margin
    out.lowerBound.z -= Margin
    out.upperBound.x += Margin
    out.upperBound.y += Margin
    out.upperBound.z += Margin
  }

  /**
   * Returns a node from the free list, or a new one.
   */
  private AllocateNode(body: Body | null): DynamicAABBNode {
    const Node = this.FreeList.pop()
    if (Node) {
      Node.Body = body
      Node.Child1 = null
      Node.Child2 = null
      Node.Parent = null
      Node.Height = 0
      return Node
    }

    const NewNode = new DynamicAABBNode()
    NewNode.Body = body
    return NewNode
  }

  /**
   * Inserts a leaf into the tree, using a surface area heuristic to pick the
   * best sibling, and rebalances the tree on the way back up.
   */
  private InsertLeaf(leaf: DynamicAABBNode): void {
    if (this.Root === null) {
      this.Root = leaf
      this.Root.Parent = null
      return
    }

    // Find the best sibling for the leaf.
    const LeafAabb = leaf.Aabb
    let Index = this.Root
    while (!Index.IsLeaf()) {
      const Child1 = Index.Child1!
      const Child2 = Index.Child2!
      const Area = SurfaceArea(Index.Aabb)
      const CombinedArea = CombinedSurfaceArea(Index.Aabb, LeafAabb)

      // Cost of creating a new parent for this node and the new leaf.
      const Cost = 2 * CombinedArea

      // Minimum cost of pushing the leaf further down the tree.
      const InheritanceCost = 2 * (CombinedArea - Area)

      // Cost of descending into Child1.
      let Cost1: number
      if (Child1.IsLeaf()) {
        Cost1 = CombinedSurfaceArea(Child1.Aabb, LeafAabb) + InheritanceCost
      } else {
        const OldArea = SurfaceArea(Child1.Aabb)
        const NewArea = CombinedSurfaceArea(Child1.Aabb, LeafAabb)
        Cost1 = NewArea - OldArea + InheritanceCost
      }

      // Cost of descending into Child2.
      let Cost2: number
      if (Child2.IsLeaf()) {
        Cost2 = CombinedSurfaceArea(Child2.Aabb, LeafAabb) + InheritanceCost
      } else {
        const OldArea = SurfaceArea(Child2.Aabb)
        const NewArea = CombinedSurfaceArea(Child2.Aabb, LeafAabb)
        Cost2 = NewArea - OldArea + InheritanceCost
      }

      // Descend according to the minimum cost.
      if (Cost < Cost1 && Cost < Cost2) {
        break
      }

      if (Cost1 < Cost2) {
        Index = Child1
      } else {
        Index = Child2
      }
    }

    const Sibling = Index

    // Create a new parent for the siblings.
    const OldParent = Sibling.Parent
    const NewParent = this.AllocateNode(null)
    NewParent.Parent = OldParent
    SetUnion(LeafAabb, Sibling.Aabb, NewParent.Aabb)
    NewParent.Height = Sibling.Height + 1

    if (OldParent !== null) {
      // The sibling was not the root.
      if (OldParent.Child1 === Sibling) {
        OldParent.Child1 = NewParent
      } else {
        OldParent.Child2 = NewParent
      }

      NewParent.Child1 = Sibling
      NewParent.Child2 = leaf
      Sibling.Parent = NewParent
      leaf.Parent = NewParent
    } else {
      // The sibling was the root.
      NewParent.Child1 = Sibling
      NewParent.Child2 = leaf
      Sibling.Parent = NewParent
      leaf.Parent = NewParent
      this.Root = NewParent
    }

    // Walk back up the tree fixing heights and AABBs.
    let Node: DynamicAABBNode | null = leaf.Parent
    while (Node !== null) {
      Node = this.Balance(Node)

      const Child1 = Node.Child1!
      const Child2 = Node.Child2!
      Node.Height = 1 + Math.max(Child1.Height, Child2.Height)
      SetUnion(Child1.Aabb, Child2.Aabb, Node.Aabb)

      Node = Node.Parent
    }
  }

  /**
   * Removes a leaf from the tree, reconnecting its sibling to the grandparent.
   */
  private RemoveLeaf(leaf: DynamicAABBNode): void {
    if (leaf === this.Root) {
      this.Root = null
      return
    }

    const Parent = leaf.Parent!
    const GrandParent = Parent.Parent
    const Sibling = Parent.Child1 === leaf ? Parent.Child2! : Parent.Child1!

    if (GrandParent !== null) {
      // Destroy Parent and connect Sibling to GrandParent.
      if (GrandParent.Child1 === Parent) {
        GrandParent.Child1 = Sibling
      } else {
        GrandParent.Child2 = Sibling
      }
      Sibling.Parent = GrandParent

      this.FreeList.push(Parent)

      // Adjust ancestor bounds.
      let Node: DynamicAABBNode | null = GrandParent
      while (Node !== null) {
        Node = this.Balance(Node)

        const Child1 = Node.Child1!
        const Child2 = Node.Child2!
        SetUnion(Child1.Aabb, Child2.Aabb, Node.Aabb)
        Node.Height = 1 + Math.max(Child1.Height, Child2.Height)

        Node = Node.Parent
      }
    } else {
      // Parent was the root, so Sibling becomes the new root.
      this.Root = Sibling
      Sibling.Parent = null

      this.FreeList.push(Parent)
    }

    // Detach the leaf so it can be reused later.
    leaf.Parent = null
    leaf.Child1 = null
    leaf.Child2 = null
    leaf.Height = 0
  }

  /**
   * Rotates the tree around the given node when its children are unbalanced.
   * @return The new root of the rotated subtree.
   */
  private Balance(a: DynamicAABBNode): DynamicAABBNode {
    if (a.IsLeaf() || a.Height < 2) {
      return a
    }

    const B = a.Child1!
    const C = a.Child2!
    const Balance = C.Height - B.Height

    // Rotate C up.
    if (Balance > 1) {
      const F = C.Child1!
      const G = C.Child2!

      // Swap A and C.
      C.Child1 = a
      C.Parent = a.Parent
      a.Parent = C

      // A's old parent should point to C.
      if (C.Parent !== null) {
        if (C.Parent.Child1 === a) {
          C.Parent.Child1 = C
        } else {
          C.Parent.Child2 = C
        }
      } else {
        this.Root = C
      }

      // Rotate.
      if (F.Height > G.Height) {
        C.Child2 = F
        a.Child2 = G
        G.Parent = a

        SetUnion(a.Child1!.Aabb, G.Aabb, a.Aabb)
        SetUnion(a.Aabb, F.Aabb, C.Aabb)

        a.Height = 1 + Math.max(a.Child1!.Height, a.Child2!.Height)
        C.Height = 1 + Math.max(a.Height, F.Height)
      } else {
        C.Child2 = G
        a.Child2 = F
        F.Parent = a

        SetUnion(a.Child1!.Aabb, F.Aabb, a.Aabb)
        SetUnion(a.Aabb, G.Aabb, C.Aabb)

        a.Height = 1 + Math.max(a.Child1!.Height, a.Child2!.Height)
        C.Height = 1 + Math.max(a.Height, G.Height)
      }

      return C
    }

    // Rotate B up.
    if (Balance < -1) {
      const D = B.Child1!
      const E = B.Child2!

      // Swap A and B.
      B.Child1 = a
      B.Parent = a.Parent
      a.Parent = B

      // A's old parent should point to B.
      if (B.Parent !== null) {
        if (B.Parent.Child1 === a) {
          B.Parent.Child1 = B
        } else {
          B.Parent.Child2 = B
        }
      } else {
        this.Root = B
      }

      // Rotate.
      if (D.Height > E.Height) {
        B.Child2 = D
        a.Child1 = E
        E.Parent = a

        SetUnion(a.Child2!.Aabb, E.Aabb, a.Aabb)
        SetUnion(a.Aabb, D.Aabb, B.Aabb)

        a.Height = 1 + Math.max(a.Child1!.Height, a.Child2!.Height)
        B.Height = 1 + Math.max(a.Height, D.Height)
      } else {
        B.Child2 = E
        a.Child1 = D
        D.Parent = a

        SetUnion(a.Child2!.Aabb, D.Aabb, a.Aabb)
        SetUnion(a.Aabb, E.Aabb, B.Aabb)

        a.Height = 1 + Math.max(a.Child1!.Height, a.Child2!.Height)
        B.Height = 1 + Math.max(a.Height, E.Height)
      }

      return B
    }

    return a
  }
}
