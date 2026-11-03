"""Meaningful numerical, sensor and ground-truth-boundary regression tests."""
import sys,unittest,ast
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from run_sim import *
class PipelineTests(unittest.TestCase):
    def test_jacobian_and_covariance(self):
        x=np.array([.2,-.1,.7]); ds=.04; da=.03; eps=1e-6
        J=np.column_stack([(motion(x+np.eye(3)[i]*eps,ds,da)-motion(x-np.eye(3)[i]*eps,ds,da))/(2*eps) for i in range(3)])
        F=np.eye(3); F[0,2]=-ds*np.sin(x[2]+da/2); F[1,2]=ds*np.cos(x[2]+da/2)
        np.testing.assert_allclose(J,F,atol=1e-7)
        e=EKF(); e.x[2]=np.pi-.01; e.P=np.diag([.01,.01,.01]); e.update(np.array([0,0,-np.pi+.01]))
        self.assertEqual(e.updates,1); self.assertGreater(np.linalg.eigvalsh(e.P).min(),0)
        before=e.rejected; e.update(np.array([100,100,0.])); self.assertEqual(e.rejected,before+1)
    def test_lqr_rank_and_stability(self):
        c=FeedbackController(); C=np.hstack([c.B,c.A@c.B,c.A@c.A@c.B])
        self.assertEqual(np.linalg.matrix_rank(C),3); self.assertLess(max(abs(c.eigenvalues)),1)
        self.assertLess(np.max(abs(c.A.T@c.P@c.A-c.A.T@c.P@c.B@c.K+c.Q-c.P)),1e-7)
        # At v_ref=0, lateral position is uncontrollable in this linearisation.
        A=np.eye(3); self.assertEqual(np.linalg.matrix_rank(np.hstack([c.B,A@c.B,A@A@c.B])),2)
    def test_controller_cannot_access_simulator(self):
        source=ast.parse((ROOT/'run_sim.py').read_text())
        c=next(n for n in source.body if isinstance(n,ast.ClassDef) and n.name=='FeedbackController')
        names={n.attr for n in ast.walk(c) if isinstance(n,ast.Attribute)}
        self.assertTrue(names.isdisjoint({'qpos','qvel','xpos','xmat','eval','sensors','d','m'}))
        demo=next(n for n in source.body if isinstance(n,ast.ClassDef) and n.name=='Demo')
        self.assertNotIn('qvel',{n.attr for n in ast.walk(demo) if isinstance(n,ast.Attribute)})
    def test_brake_reset_and_charge_interlock(self):
        d=Demo()
        try:
            d.d.time=1.; d.steps=1; d.planner.path=[np.array([.4,0.]),np.array([1.,0.])]; d.ranges[:]=.1; d.tick()
            self.assertEqual(d.stage,'LIDAR BRAKE'); self.assertAlmostEqual(sum(d.targets),0.)
            d.reset(); d.tick(); self.assertEqual(d.index,0); self.assertEqual(d.ekf.updates,1)
            d.d.time=1.; d.steps=1; d.ekf.x=np.array([-.498,0.,np.pi]); d.dock=np.array([-.698,0.]); d.stage='CHARGING'; d.charging_since=0.; d.station_age=1.; d.camera_age=1.; d.sensors.last=d.d.qpos[d.sensors.qadr].copy()
            d.tick(); self.assertTrue(d.battery.connected)
            d.station_age=-5.; d.tick(); self.assertFalse(d.battery.connected)
        finally: d.sensors.close()
    def test_stale_navigation_dock_is_reacquired(self):
        d=Demo()
        try:
            self.assertEqual(d.goals.tolist(),[[1.,0.],[0.,1.],[0.,0.]])
            d.d.time=1.; d.steps=1; d.index=len(d.goals)
            d.dock=np.array([-.73,-.04]); d.station_age=-5.; d.stage='BLOCKED'
            d.tick()
            self.assertIsNone(d.dock); self.assertEqual(d.stage,'SEARCH DOCK')
            self.assertAlmostEqual(sum(d.targets),0.)
        finally: d.sensors.close()
    def test_charge_demonstration_finishes_after_one_second(self):
        d=Demo()
        try:
            d.stage='CHARGING'; d.charging_since=10.; d.d.time=10.99
            self.assertFalse(d.complete())
            d.d.time=11.; self.assertTrue(d.complete())
            d.stage='DOCK ALIGN'; self.assertFalse(d.complete())
        finally: d.sensors.close()
if __name__=='__main__': unittest.main()
