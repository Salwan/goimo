package demos

// ///////////////////////////////////////// PgsContactConstraintSolver
// (oimo/dynamics/constraint/solver/pgs/PgsContactConstraintSolver.go)
// A contact constraint solver using projected Gauss-Seidel (sequential impulse).
type PgsContactConstraintSolver struct {
	*ConstraintSolver

	constraint *ContactConstraint

	info *ContactSolverInfo

	massData []*ContactSolverMassDataRow
}

func NewPgsContactConstraintSolver(constraint *ContactConstraint) *PgsContactConstraintSolver {
	pgs := &PgsContactConstraintSolver{
		ConstraintSolver: NewConstraintSolver(),
		constraint:       constraint,
		info:             NewContactSolverInfo(),
		massData:         make([]*ContactSolverMassDataRow, Settings.MaxManifoldPoints),
	}
	for i := range len(pgs.massData) {
		pgs.massData[i] = NewContactSolverMassDataRow()
	}
	return pgs
}

func (self *PgsContactConstraintSolver) _updatePositionData() {
	self.constraint.syncManifold()
	self.constraint.getPositionSolverInfo(self.info)

	invM1 := self.b1.invMass
	invM2 := self.b2.invMass

	invI1 := self.b1.invInertia
	invI2 := self.b2.invInertia

	// compute mass data
	for i := range self.info.numRows {
		row := self.info.rows[i]
		md := self.massData[i]
		j := row.jacobianN

		md.invMLinN1 = j.lin1.Scale(invM1)
		md.invMLinN2 = j.lin2.Scale(invM2)
		md.invMAngN1 = j.ang1.MulMat3(&invI1)
		md.invMAngN2 = j.ang2.MulMat3(&invI2)

		md.massN = invM1 + invM2 + md.invMAngN1.Dot(j.ang1) + md.invMAngN2.Dot(j.ang2)
		if md.massN != 0 {
			md.massN = 1.0 / md.massN
		}
	}
}

func (self *PgsContactConstraintSolver) PreSolveVelocity(timeStep TimeStep) {
	self.constraint.getVelocitySolverInfo(timeStep, self.info)

	self.b1 = self.info.b1
	self.b2 = self.info.b2

	invM1 := self.b1.invMass
	invM2 := self.b2.invMass

	invI1 := self.b1.invInertia
	invI2 := self.b2.invInertia

	// compute mass data
	for i := range self.info.numRows {
		row := self.info.rows[i]
		md := self.massData[i]

		// normal mass
		j := row.jacobianN
		md.invMLinN1 = j.lin1.Scale(invM1)
		md.invMLinN2 = j.lin2.Scale(invM2)
		md.invMAngN1 = j.ang1.MulMat3(&invI1)
		md.invMAngN2 = j.ang2.MulMat3(&invI2)

		md.massN = invM1 + invM2 + md.invMAngN1.Dot(j.ang1) + md.invMAngN2.Dot(j.ang2)
		if md.massN != 0 {
			md.massN = 1.0 / md.massN
		}

		// tangent/binormal mass
		jt := row.jacobianT
		jb := row.jacobianB
		md.invMLinT1 = jt.lin1.Scale(invM1)
		md.invMLinT2 = jt.lin2.Scale(invM2)
		md.invMLinB1 = jb.lin1.Scale(invM1)
		md.invMLinB2 = jb.lin2.Scale(invM2)
		md.invMAngT1 = jt.ang1.MulMat3(&invI1)
		md.invMAngT2 = jt.ang2.MulMat3(&invI2)
		md.invMAngB1 = jb.ang1.MulMat3(&invI1)
		md.invMAngB2 = jb.ang2.MulMat3(&invI2)

		// compute effective mass matrix for friction
		invMassTB00 := invM1 + invM2 + md.invMAngT1.Dot(jt.ang1) + md.invMAngT2.Dot(jt.ang2)
		invMassTB01 := md.invMAngT1.Dot(jb.ang1) + md.invMAngT2.Dot(jb.ang2)
		invMassTB10 := invMassTB01
		invMassTB11 := invM1 + invM2 + md.invMAngB1.Dot(jb.ang1) + md.invMAngB2.Dot(jb.ang2)

		invDet := invMassTB00*invMassTB11 - invMassTB01*invMassTB10
		if invDet != 0 {
			invDet = 1.0 / invDet
		}

		md.massTB00 = invMassTB11 * invDet
		md.massTB01 = -invMassTB01 * invDet
		md.massTB10 = -invMassTB10 * invDet
		md.massTB11 = invMassTB00 * invDet
	}
}

func (self *PgsContactConstraintSolver) WarmStart(timeStep TimeStep) {
	lv1 := self.b1.vel
	lv2 := self.b2.vel
	av1 := self.b1.angVel
	av2 := self.b2.angVel

	for i := range self.info.numRows {
		row := self.info.rows[i]
		imp := row.impulse
		md := self.massData[i]
		jt := row.jacobianT
		jb := row.jacobianB

		impulseN := imp.impulseN
		impulseT := imp.impulseL.Dot(jt.lin1)
		impulseB := imp.impulseL.Dot(jb.lin1)
		imp.impulseT = impulseT
		imp.impulseB = impulseB

		// adjust impulse for variable time step
		imp.impulseN *= timeStep.DtRatio
		imp.impulseT *= timeStep.DtRatio
		imp.impulseB *= timeStep.DtRatio

		lv1 = lv1.AddRhsScaled(md.invMLinN1, impulseN)
		lv1 = lv1.AddRhsScaled(md.invMLinT1, impulseT)
		lv1 = lv1.AddRhsScaled(md.invMLinB1, impulseB)
		lv2 = lv2.AddRhsScaled(md.invMLinN2, -impulseN)
		lv2 = lv2.AddRhsScaled(md.invMLinT2, -impulseT)
		lv2 = lv2.AddRhsScaled(md.invMLinB2, -impulseB)
		av1 = av1.AddRhsScaled(md.invMAngN1, impulseN)
		av1 = av1.AddRhsScaled(md.invMAngT1, impulseT)
		av1 = av1.AddRhsScaled(md.invMAngB1, impulseB)
		av2 = av2.AddRhsScaled(md.invMAngN2, -impulseN)
		av2 = av2.AddRhsScaled(md.invMAngT2, -impulseT)
		av2 = av2.AddRhsScaled(md.invMAngB2, -impulseB)
	}

	self.b1.vel = lv1
	self.b2.vel = lv2
	self.b1.angVel = av1
	self.b2.angVel = av2
}

func (self *PgsContactConstraintSolver) SolveVelocity() {
	lv1 := self.b1.vel
	lv2 := self.b2.vel
	av1 := self.b1.angVel
	av2 := self.b2.angVel

	// solve friction
	for i := range self.info.numRows {
		row := self.info.rows[i]
		md := self.massData[i]
		imp := row.impulse

		// measure relative velocity
		rvt := 0.0
		j := row.jacobianT
		rvt += lv1.Dot(j.lin1)
		rvt -= lv2.Dot(j.lin2)
		rvt += av1.Dot(j.ang1)
		rvt -= av2.Dot(j.ang2)

		rvb := 0.0
		j = row.jacobianB
		rvb += lv1.Dot(j.lin1)
		rvb -= lv2.Dot(j.lin2)
		rvb += av1.Dot(j.ang1)
		rvb -= av2.Dot(j.ang2)

		impulseT := -(rvt*md.massTB00 + rvb*md.massTB01)
		impulseB := -(rvt*md.massTB10 + rvb*md.massTB11)
		oldImpulseT := imp.impulseT
		oldImpulseB := imp.impulseB
		imp.impulseT += impulseT
		imp.impulseB += impulseB

		// cone friction
		maxImpulse := row.friction * imp.impulseN
		if maxImpulse == 0 {
			imp.impulseT = 0
			imp.impulseB = 0
		} else {
			impulseLengthSq := imp.impulseT*imp.impulseT + imp.impulseB*imp.impulseB
			if impulseLengthSq > maxImpulse*maxImpulse {
				invL := maxImpulse / MathUtil.Sqrt(impulseLengthSq)
				imp.impulseT *= invL
				imp.impulseB *= invL
			}
		}

		impulseT = imp.impulseT - oldImpulseT
		impulseB = imp.impulseB - oldImpulseB

		// apply delta impulse
		lv1 = lv1.AddRhsScaled(md.invMLinT1, impulseT)
		lv1 = lv1.AddRhsScaled(md.invMLinB1, impulseB)
		lv2 = lv2.AddRhsScaled(md.invMLinT2, -impulseT)
		lv2 = lv2.AddRhsScaled(md.invMLinB2, -impulseB)
		av1 = av1.AddRhsScaled(md.invMAngT1, impulseT)
		av1 = av1.AddRhsScaled(md.invMAngB1, impulseB)
		av2 = av2.AddRhsScaled(md.invMAngT2, -impulseT)
		av2 = av2.AddRhsScaled(md.invMAngB2, -impulseB)
	}

	// solve normal
	for i := range self.info.numRows {
		row := self.info.rows[i]
		md := self.massData[i]
		imp := row.impulse

		// measure relative velocity
		rvn := 0.0
		j := row.jacobianN
		rvn += lv1.Dot(j.lin1)
		rvn -= lv2.Dot(j.lin2)
		rvn += av1.Dot(j.ang1)
		rvn -= av2.Dot(j.ang2)

		impulseN := (row.rhs - rvn) * md.massN

		// clamp impulse
		oldImpulseN := imp.impulseN
		imp.impulseN += impulseN
		if imp.impulseN < 0 {
			imp.impulseN = 0
		}
		impulseN = imp.impulseN - oldImpulseN

		// apply delta impulse
		lv1 = lv1.AddRhsScaled(md.invMLinN1, impulseN)
		lv2 = lv2.AddRhsScaled(md.invMLinN2, -impulseN)
		av1 = av1.AddRhsScaled(md.invMAngN1, impulseN)
		av2 = av2.AddRhsScaled(md.invMAngN2, -impulseN)
	}

	self.b1.vel = lv1
	self.b2.vel = lv2
	self.b1.angVel = av1
	self.b2.angVel = av2
}

func (self *PgsContactConstraintSolver) PostSolveVelocity(timeStep TimeStep) {
	// Not implemented in Oimo
}

func (self *PgsContactConstraintSolver) PreSolvePosition(timeStep TimeStep) {
	self._updatePositionData()

	// clear position impulses
	for i := range self.info.numRows {
		self.info.rows[i].impulse.impulseP = 0
	}
}

func (self *PgsContactConstraintSolver) SolvePositionSplitImpulse() {
	lv1 := self.b1.pseudoVel
	lv2 := self.b2.pseudoVel
	av1 := self.b1.angPseudoVel
	av2 := self.b2.angPseudoVel

	// solve normal
	for i := range self.info.numRows {
		row := self.info.rows[i]
		md := self.massData[i]
		imp := row.impulse
		j := row.jacobianN

		// measure relative velocity
		rvn := 0.0
		rvn += lv1.Dot(j.lin1)
		rvn -= lv2.Dot(j.lin2)
		rvn += av1.Dot(j.ang1)
		rvn -= av2.Dot(j.ang2)

		impulseP := (row.rhs - rvn) * md.massN * Settings.PositionSplitImpulseBaumgarte

		// clamp impulse
		oldImpulseP := imp.impulseP
		imp.impulseP += impulseP
		if imp.impulseP < 0 {
			imp.impulseP = 0
		}
		impulseP = imp.impulseP - oldImpulseP

		// apply delta impulse
		lv1 = lv1.AddRhsScaled(md.invMLinN1, impulseP)
		lv2 = lv2.AddRhsScaled(md.invMLinN2, -impulseP)
		av1 = av1.AddRhsScaled(md.invMAngN1, impulseP)
		av2 = av2.AddRhsScaled(md.invMAngN2, -impulseP)
	}

	self.b1.pseudoVel = lv1
	self.b2.pseudoVel = lv2
	self.b1.angPseudoVel = av1
	self.b2.angPseudoVel = av2
}

func (self *PgsContactConstraintSolver) SolvePositionNgs(timeStep TimeStep) {
	self._updatePositionData()

	var lv1, lv2, av1, av2 Vec3

	for i := range self.info.numRows {
		row := self.info.rows[i]
		md := self.massData[i]
		imp := row.impulse
		j := row.jacobianN

		// estimate translation along the normal
		rvn := 0.0
		rvn += lv1.Dot(j.lin1)
		rvn -= lv2.Dot(j.lin2)
		rvn += av1.Dot(j.ang1)
		rvn -= av2.Dot(j.ang2)

		impulseP := (row.rhs - rvn) * md.massN * Settings.PositionNgsBaumgarte

		// clamp impulse
		oldImpulseP := imp.impulseP
		imp.impulseP += impulseP
		if imp.impulseP < 0 {
			imp.impulseP = 0
		}
		impulseP = imp.impulseP - oldImpulseP

		// apply delta impulse
		lv1 = lv1.AddRhsScaled(md.invMLinN1, impulseP)
		lv2 = lv2.AddRhsScaled(md.invMLinN2, -impulseP)
		av1 = av1.AddRhsScaled(md.invMAngN1, impulseP)
		av2 = av2.AddRhsScaled(md.invMAngN2, -impulseP)
	}

	self.b1.applyTranslation(lv1)
	self.b2.applyTranslation(lv2)
	self.b1.applyRotation(av1)
	self.b2.applyRotation(av2)
}

func (self *PgsContactConstraintSolver) PostSolve() {
	// contact impulses
	var lin1, ang1, ang2 Vec3

	for i := range self.info.numRows {
		row := self.info.rows[i]
		imp := row.impulse
		jn := row.jacobianN
		jt := row.jacobianT
		jb := row.jacobianB
		impN := imp.impulseN
		impT := imp.impulseT
		impB := imp.impulseB
		var impulseL Vec3

		// store lateral impulse
		impulseL = impulseL.AddRhsScaled(jt.lin1, impT)
		impulseL = impulseL.AddRhsScaled(jb.lin1, impB)
		imp.impulseL = impulseL

		// accumulate contact impulses
		lin1 = lin1.AddRhsScaled(jn.lin1, impN)
		ang1 = ang1.AddRhsScaled(jn.ang1, impN)
		ang2 = ang2.AddRhsScaled(jn.ang2, impN)
		lin1 = lin1.AddRhsScaled(jt.lin1, impT)
		ang1 = ang1.AddRhsScaled(jt.ang1, impT)
		ang2 = ang2.AddRhsScaled(jt.ang2, impT)
		lin1 = lin1.AddRhsScaled(jb.lin1, impB)
		ang1 = ang1.AddRhsScaled(jb.ang1, impB)
		ang2 = ang2.AddRhsScaled(jb.ang2, impB)
	}

	self.b1.linearContactImpulse = self.b1.linearContactImpulse.Add(lin1)
	self.b1.angularContactImpulse = self.b1.angularContactImpulse.Add(ang1)
	self.b2.linearContactImpulse = self.b2.linearContactImpulse.Sub(lin1)
	self.b2.angularContactImpulse = self.b2.angularContactImpulse.Sub(ang2)

	self.constraint.syncManifold()
}
