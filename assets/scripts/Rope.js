let SIDE_LEFT = -1
let SIDE_RIGTH = 1
let MAX_SPEED = 400
cc.Class({
    extends: cc.Component,

    properties: {
        loopNode: cc.Node,
    },

    onLoad() {
        //设置物理引擎
        let manager = cc.director.getPhysicsManager();
        manager.enabled = true;
        manager.enabledAccumulator = true;
        // manager.gravity = cc.v2(0, 0);

        // cc.director.getPhysicsManager().debugDrawFlags = cc.PhysicsManager.DrawBits.e_shapeBit |
        //     cc.PhysicsManager.DrawBits.e_pairBit |
        //     cc.PhysicsManager.DrawBits.e_centerOfMassBit |
        //     cc.PhysicsManager.DrawBits.e_jointBit |
        //     cc.PhysicsManager.DrawBits.e_aabbBit
        //添加渲染
        let gNode = new cc.Node("gNode");
        this.ctx = gNode.addComponent(cc.Graphics);
        this.ctx.lineWidth = 10
        this.ctx.strokeColor = cc.color().fromHEX("#FFFFFF")
        this.node.addChild(gNode)
        //初始化对象
        this.node.rope = this.node.getComponent(cc.RopeJoint);
        this.loopNode.body = this.loopNode.getComponent(cc.RigidBody)
        this.loopNode.circleCollider = this.loopNode.getComponent(cc.PhysicsCircleCollider)
        //添加鼠标移动
        let joint = this.loopNode.addComponent(cc.MouseJoint);
        joint.mouseRegion = this.loopNode;
        joint.apply();
        this.resetData()
        this.ropeMaxLength = this.node.rope.maxLength
    },

    resetData(){
        //初始化参数
        this._pivots = []

        this.loopNode.circleRadius = this.loopNode.circleCollider.radius
        this.node.rope.anchor = cc.v2(0,0)
        this.node.rope.apply()
        this.m_startPos = { point: this.node.parent.convertToWorldSpaceAR(this.node.rope.anchor) }
        this.m_endPos = { point: this.node.parent.convertToWorldSpaceAR(this.loopNode.position) }
        this.m_lastPivot = {}
        this.m_lastPivot.point = this.m_startPos.point
    },

    updateRopeLength() { //更新绳子的长度
        this.node.rope.maxLength = this.ropeMaxLength
        // console.log("绳子长度：",this.node.rope.maxLength)
        this.node.rope.apply()
    },

    addPivot(p, collider) {    //添加轴心点
        let pivot = {}
        pivot.collider = collider
        pivot.point = p
        pivot.pivotDirection = b2.Vec2.SubVV(p, this.m_lastPivot.point, cc.v2(0, 0))   //线段方向
        pivot.sideOfObstacle = this.getSideOfObstacle(collider, pivot.pivotDirection) //线段所在碰撞对象collider的方向
        this._pivots.push(pivot)

        this.ropeMaxLength -= this.m_lastPivot.point.sub(p).mag()   //更新绳子的长度
        this.updateRopeLength()

        this.m_lastPivot = pivot
        let pos = this.node.convertToNodeSpaceAR(pivot.point)    //将新的检测点转换为绳子的本地坐标
        this.node.rope.anchor = pos
        this.node.rope.apply()
        // console.log(this.ropeMaxLength)
    },

    getSideOfObstacle(collider, direction) {  //线段所在碰撞对象collider的方向
        let centerOfObstacle = collider.body.getWorldCenter()
        let obstacleDirection = b2.Vec2.SubVV(centerOfObstacle, this.m_lastPivot.point, cc.v2(0, 0))
        let sideOfObstacle = b2.Vec2.CrossVV(obstacleDirection, direction)
        if (sideOfObstacle > 0) {
            return SIDE_LEFT
        } else {
            return SIDE_RIGTH
        }
    },

    removeLastPivot() {  //删除轴心点
        let removePivots = this.m_endPos
        removePivots = this._pivots.pop()
        if (this._pivots.length) {  //有两个以上拐点
            this.m_lastPivot = this._pivots[this._pivots.length - 1]
        } else {
            this.m_lastPivot = this.m_startPos
        }

        this.ropeMaxLength += this.m_lastPivot.point.sub(removePivots.point).mag()  //更新绳子的长度
        this.updateRopeLength()

        this.node.rope.anchor = this.node.convertToNodeSpaceAR(this.m_lastPivot.point)
        this.node.rope.apply()
        // console.log(this.ropeMaxLength)
    },

    getVertices() {  //获取所有顶点
        let Vertices = [this.m_startPos].concat(this._pivots, [this.m_endPos])
        return Vertices
    },

    excludeResults(results) {
        let self = this
        let newResults = results.filter(function (elem) {
            if (elem.collider.tag == 0) {
                return false
            }
            if (elem.collider.node.uuid == self.node.uuid) {
                return false
            }
            return true
        })
        return newResults
    },

    getLoopNodeLinkLinePoint(){ //获取环上连线的点
        let pos = this.node.convertToWorldSpaceAR(this.loopNode.position)   //圆心
        let angle = Math.atan2(pos.y - this.m_lastPivot.point.y, pos.x - this.m_lastPivot.point.x);  //得到弧度
        angle = 180 + angle * 180 / Math.PI;     //得到角度
        let linkLinePos = cc.v2(Math.cos(Math.PI / 180 * angle), Math.sin(Math.PI / 180 * angle)).mulSelf(this.loopNode.circleRadius-5).addSelf(pos)
        return linkLinePos
    },

    update(dt) {
        let endPos = this.getLoopNodeLinkLinePoint()
        let centerPos = this.node.convertToWorldSpaceAR(this.loopNode.position)
        this.m_endPos.point = endPos
        let backswing = false
        //回摆处理
        if (this._pivots.length) {
            let direction = b2.Vec2.SubVV(centerPos, this.m_lastPivot.point, cc.v2(0, 0))   //线段方向
            let sideOfLoopNode = b2.Vec2.CrossVV(direction, this.m_lastPivot.pivotDirection)
            if (sideOfLoopNode * this.m_lastPivot.sideOfObstacle > 0) {
                backswing = true
                this.removeLastPivot()
            }
        }    
        //射线处理
        if (!backswing) {   //有回摆的时候不处理射线
            //将射线基于原线段延长一段，如果把射线起点直接设置将会检测不到
            let angle = Math.atan2(centerPos.y - this.m_lastPivot.point.y, centerPos.x - this.m_lastPivot.point.x);  //得到弧度
            angle = 180 + angle * 180 / Math.PI;     //得到角度
            let p1 = cc.v2(Math.cos(Math.PI / 180 * angle), Math.sin(Math.PI / 180 * angle)).mulSelf(10).addSelf(this.m_lastPivot.point);
            //rayCast射线检测
            let manager = cc.director.getPhysicsManager();
            let results = manager.rayCast(centerPos, p1, cc.RayCastType.Closest);
            if (results[0]) {   //射线有新的检测点
                results = this.excludeResults(results)
                if (results.length) {
                    let newPivot = results[results.length - 1].point;
                    if (this.m_lastPivot.point.sub(newPivot).mag() > 1) {  //距离大于2像素才更新，密度过大可能导致性能问题
                        this.addPivot(newPivot, results[results.length - 1].collider)
                    }
                }
            }
            //最大速度
            let speed = this.loopNode.body.linearVelocity
            if (Math.abs(speed.x) > MAX_SPEED) {
                speed.x = MAX_SPEED * speed.x / Math.abs(speed.x)
            }
            if (Math.abs(speed.y) > MAX_SPEED) {
                speed.y = MAX_SPEED * speed.y / Math.abs(speed.y)
            }
            this.loopNode.body.linearVelocity = speed
        }
    },

    lateUpdate(dt){
        this.drawLine()
    },

    drawLine() {    //渲染出绳子轨迹
        this.ctx.clear();
        let Vertices = this.getVertices()
        let start = this.node.convertToNodeSpaceAR(Vertices.shift().point)
        this.ctx.moveTo(start.x, start.y)
        while (Vertices.length) {
            let pivot = this.node.convertToNodeSpaceAR(Vertices.shift().point)
            this.ctx.lineTo(pivot.x, pivot.y)
        }
        this.ctx.stroke();
    },
});
