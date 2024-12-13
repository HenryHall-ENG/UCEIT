import numpy as np
import matplotlib.pyplot as plt
import queue
import matplotlib.colors as col
import pyeit
import pyeit.mesh as mesh
import pyeit.eit.protocol as protocol
from pyeit.eit.fem import EITForward
from pyeit.eit.greit import GREIT

class Forward():
    def __init__(self,r,n_el,perm,h0):
        self.r = r
        self.n_el = n_el
        self.perm = perm
        self.h0 = h0

        self.shape = self._fd

        self.meshing()
        self.protocol()
    
    def _fd(self, pts):
        return pyeit.mesh.shape.circle(pts, pc=[0, 0], r=self.r)
    
    def meshing(self):
        self.mesh_obj = mesh.create(self.n_el, fd=self.shape, h0=self.h0)
        self.mesh_obj.perm=self.perm

    def protocol(self):
        self.protocol_obj = protocol.create(self.mesh_obj.n_el, dist_exc=1, step_meas=1, parser_meas="std")

    def forwardSolve(self):
        fwd = EITForward(self.mesh_obj, self.protocol_obj)
        v0 = fwd.solve_eit()
        return v0

    

class Inverse():
    def __init__(self,p,lam,perm,forward):
        self.p = p
        self.lam = lam
        self.perm = perm
        self.forward = forward

        self.initGREIT()

    def initGREIT(self):
        self.eit = GREIT(self.forward.mesh_obj, self.forward.protocol_obj)
        self.eit.setup(p=0.50, lamb=0.01, perm=1, jac_normalized=True)

    def inverseSolve(self, data1, data2):
        ds = self.eit.solve(np.array(data2), np.array(data1), normalize=True)
        x, y, ds = self.eit.mask_value(ds, mask_value=np.NAN)
        return ds


class EIT():
    def __init__(self,r,n_el,perm,h0,p,lam):
        self.forward = Forward(r,n_el,perm,h0)
        self.inverse = Inverse(p,lam,perm,self.forward)

        self.ref = None
        self.is_running = False
        
    def dataGet(self, queue):
        self.data = queue.get()

    def setRef(self):
        # self.dataGet(queue)
        self.ref = self.data
        self.is_running = True

    def stop(self):
        self.ref = None
        self.is_running = False

    def updateForward(self,r,n_el,perm,h0):
        self.forward = Forward(r,n_el,perm,h0)

    def updateInverse(self,p,lam,perm):
        self.inverse = Inverse(p,lam,perm,self.forward)

    def drawMesh(self, fig, axes):
        pts = self.forward.mesh_obj.node
        tri = self.forward.mesh_obj.element
        el_pos = self.forward.mesh_obj.el_pos
        axes.cla()

        axes.triplot(pts[:, 0], pts[:, 1], tri, linewidth=1)
        axes.plot(pts[el_pos, 0], pts[el_pos, 1], "ro")
        axes.set_title('Mesh in Use')
        return fig,axes
    
    def drawForward(self, fig, axes):
        v0 = self.forward.forwardSolve()
        axes.cla()

        axes.plot(np.arange(0,len(v0), 1), v0)
        axes.set_title('Forward Model Solution')
        axes.grid(True)
        axes.set_xlim(-1)
        return fig, axes
    
    def displayEIT(self, fig, axes):
        ds = None
        if self.is_running:
            ds = self.inverse.inverseSolve(self.data, self.ref)
            axes.cla()

            
            im = axes.imshow(np.real(ds), interpolation="none", cmap=plt.cm.plasma, norm=col.PowerNorm(gamma=1.2, vmin=0, vmax=1))
            print(np.max(np.nan_to_num(ds,nan=0)))
            axes.axis("equal")
            # fig.colorbar(im, ax=axes.ravel().tolist())

        return fig, axes
    
    def displayMeas(self, fig, axes):
        axes.cla()
        axes.plot(np.arange(0,len(self.data),1),self.data)
        if self.is_running:
            axes.plot(np.arange(0,len(self.ref),1),self.ref, '--')
        axes.set_title('Live Measurements')
        axes.grid(True)
        axes.set_xlim(-1)
        return fig, axes


