import rerun as rr
import scipy.spatial.transform as st
from urdf_parser_py import urdf as urdf_parser
import trimesh
from pathlib import Path
from PIL import Image
import numpy as np


class URDFLogger:
    def __init__(self, urdf_path: str, entity_prefix: str = ""):
        self.urdf_path = urdf_path
        self.entity_prefix = entity_prefix
        self.urdf = urdf_parser.URDF.from_xml_file(urdf_path)
        self.root_path = Path(self.urdf_path).parent

    def link_entity_path(self, link: urdf_parser.Link) -> str:
        """Return the entity path for the URDF link."""
        root_name = self.urdf.get_root()
        link_names = self.urdf.get_chain(root_name, link.name)[0::2]  # skip the joints
        return (
            self.entity_prefix + ("/".join(link_names))
            if self.entity_prefix
            else "/" + ("/".join(link_names))
        )

    def joint_entity_path(self, joint: urdf_parser.Joint) -> str:
        """Return the entity path for the URDF joint."""
        root_name = self.urdf.get_root()
        joint_names = self.urdf.get_chain(root_name, joint.child)[
            0::2
        ]  # skip the links
        return (
            self.entity_prefix + ("/".join(joint_names))
            if self.entity_prefix
            else "/" + ("/".join(joint_names))
        )

    def log(self, joint_stats: str = "Original") -> None:
        """Log a URDF file to Rerun."""

        if joint_stats == "Original":
            for joint in self.urdf.joints:
                entity_path = self.joint_entity_path(joint)
                self.log_joint(entity_path, joint)

            for link in self.urdf.links:
                entity_path = self.link_entity_path(link)
                self.log_link(entity_path, link)

        else:
            for i, joint in enumerate(self.urdf.joints):
                from scipy.spatial.transform import Rotation as R
                R_rand = R.random().as_matrix()
                entity_path = self.joint_entity_path(joint)
                rr.log(entity_path, R_rand)

    def log_link(self, entity_path: str, link: urdf_parser.Link) -> None:
        """Log a URDF link to Rerun."""

        # create one mesh out of all visuals
        for i, visual in enumerate(link.visuals):
            self.log_visual(entity_path + f"/visual_{i}", visual)

    @staticmethod
    def origin_to_transform(origin: urdf_parser.Pose | None) -> rr.Transform3D | None:
        """Convert a URDF origin to a Rerun transform."""
        if origin is None:
            return None

        if origin.xyz is not None:
            translation = origin.xyz
        if origin.rpy is not None:
            rotation = st.Rotation.from_euler("xyz", origin.rpy).as_quat()

        if translation is not None and rotation is not None:
            return rr.Transform3D(translation=translation, quaternion=rotation)
        else:
            return None

    @staticmethod
    def _joint_motion_transform(
        joint: urdf_parser.Joint, value: float
    ) -> rr.Transform3D | None:
        """Transform produced by joint motion value (rad or m)."""
        jt = joint.type
        axis = (
            np.array(joint.axis)
            if joint.axis is not None
            else np.array([1.0, 0.0, 0.0])
        )
        if jt in ("revolute", "continuous"):
            rot = st.Rotation.from_rotvec(axis / np.linalg.norm(axis) * value).as_quat()
            return rr.Transform3D(quaternion=rot)
        if jt == "prismatic":
            disp = axis / np.linalg.norm(axis) * value
            return rr.Transform3D(translation=disp)
        # fixed or other types: no motion
        return rr.Transform3D()

    @staticmethod
    def _compose(
        t1: rr.Transform3D | None, t2: rr.Transform3D | None
    ) -> rr.Transform3D | None:
        """Compose two transforms (t1 then t2)."""
        if t1 is None:
            return t2
        if t2 is None:
            return t1
        # Extract parts (fallback defaults)
        trans1 = (
            np.array(t1.translation)
            if getattr(t1, "translation", None) is not None
            else np.zeros(3)
        )
        trans2 = (
            np.array(t2.translation)
            if getattr(t2, "translation", None) is not None
            else np.zeros(3)
        )
        quat1 = (
            np.array(t1.quaternion)
            if getattr(t1, "quaternion", None) is not None
            else np.array([0, 0, 0, 1])
        )
        quat2 = (
            np.array(t2.quaternion)
            if getattr(t2, "quaternion", None) is not None
            else np.array([0, 0, 0, 1])
        )

        r1 = st.Rotation.from_quat(quat1)
        r2 = st.Rotation.from_quat(quat2)
        # Compose rotation
        r = r1 * r2
        # trans2 is in frame of t1
        composed_t = trans1 + r1.apply(trans2)
        return rr.Transform3D(translation=composed_t, quaternion=r.as_quat())

    def log_joint(self, entity_path: str, joint: urdf_parser.Joint) -> None:
        """Log a URDF joint to Rerun."""
        # origin_to_transform(joint.origin)
        # if joint.origin is not None and joint.origin.xyz is not None:
        #     translation = joint.origin.xyz
        # if joint.origin is not None and joint.origin.rpy is not None:
        #     rotation = st.Rotation.from_euler("xyz", joint.origin.rpy).as_matrix()
        transform = self.origin_to_transform(joint.origin)

        if transform is not None:
            rr.log(
                entity_path,
                transform,
            )

    def log_visual(self, entity_path: str, visual: urdf_parser.Visual) -> None:
        """Log a URDF visual to Rerun."""
        material = visual.material
        transform = self.origin_to_transform(visual.origin)

        if isinstance(visual.geometry, urdf_parser.Mesh):
            resolved_path = self.root_path / visual.geometry.filename
            mesh_scale = visual.geometry.scale
            mesh_or_scene = trimesh.load_mesh(resolved_path)
            if mesh_scale is not None:
                if transform is not None:
                    transform.scale = mesh_scale
                else:
                    transform = rr.Transform3D(scale=mesh_scale)

        else:
            raise NotImplementedError("Only mesh geometries are supported.")

        if isinstance(mesh_or_scene, trimesh.Trimesh):
            mesh = mesh_or_scene
            if material is not None and not isinstance(
                mesh.visual, trimesh.visual.texture.TextureVisuals
            ):
                if material.color is not None:
                    mesh.visual = trimesh.visual.ColorVisuals()
                    mesh.visual.vertex_colors = material.color.rgba
                elif material.texture is not None:
                    texture_path = self.root_path / material.texture.filename
                    mesh.visual = trimesh.visual.texture.TextureVisuals(
                        image=Image.open(texture_path)
                    )
            self.log_trimesh(entity_path, mesh, transform)

        else:
            raise NotImplementedError("Only single mesh geometries are supported.")

    @staticmethod
    def log_trimesh(
        entity_path: str,
        mesh: trimesh.Trimesh,
        transform: rr.Transform3D | None,
    ) -> None:
        vertex_colors = albedo_texture = vertex_texcoords = None

        if isinstance(mesh.visual, trimesh.visual.color.ColorVisuals):
            vertex_colors = mesh.visual.vertex_colors
        else:
            raise NotImplementedError("Only color visuals are supported.")

        rr.log(
            entity_path,
            rr.Mesh3D(
                vertex_positions=mesh.vertices,
                triangle_indices=mesh.faces,
                vertex_normals=mesh.vertex_normals,
                vertex_colors=vertex_colors,
                albedo_texture=albedo_texture,
                vertex_texcoords=vertex_texcoords,
            ),
            static=True,
        )

        if transform is not None:
            rr.log(entity_path, transform, static=True)


if __name__ == "__main__":

    rr.init("urdf_logger_example")
    rr.spawn()

    urdf_logger = URDFLogger("so101_new_calib.urdf", entity_prefix="xx")
    urdf_logger.log(joint_stats="Original")

    for _ in range(200):
        urdf_logger.log(joint_stats="Random")

    pass
