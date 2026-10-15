// URDF Builder Module
// Shared URDF -> Three.js scene graph construction, used by the desktop robot viewer
// (robot_viewer.js) and the VR top/side view panel (vr_urdf_panel.js).
// Requires the global THREE (libs/three.min.js) and optionally THREE.OBJLoader.

const MESH_PACKAGE_PREFIX = 'package://alfie_urdf/meshes/';
const DEFAULT_COLOR = 0x666666;

function parseVec(str) {
    return str.trim().split(/\s+/).map(parseFloat);
}

/**
 * Apply a URDF <origin xyz rpy> to an Object3D
 */
export function applyOrigin(object, origin) {
    if (!origin) return;

    const xyz = origin.getAttribute('xyz');
    const rpy = origin.getAttribute('rpy');

    if (xyz) {
        const pos = parseVec(xyz);
        object.position.set(pos[0], pos[1], pos[2]);
    }
    if (rpy) {
        const rot = parseVec(rpy);
        object.rotation.order = 'ZYX';
        object.rotation.set(rot[0], rot[1], rot[2]);
    }
}

/**
 * Dispose geometries, materials and material textures under an Object3D
 */
export function disposeObject3D(root) {
    if (!root) return;
    root.traverse((object) => {
        if (object.geometry) {
            object.geometry.dispose();
        }
        if (object.material) {
            const materials = Array.isArray(object.material) ? object.material : [object.material];
            for (const material of materials) {
                for (const value of Object.values(material)) {
                    if (value && value.isTexture) value.dispose();
                }
                material.dispose();
            }
        }
    });
}

/**
 * Build a Three.js robot from a URDF string.
 * @param {string} urdfString - The URDF XML string
 * @param {Object} [opts]
 * @param {Function} [opts.log] - Logger
 * @returns {{robot: THREE.Group, name: string, links: Object, joints: Object, dispose: Function}|null}
 */
export function buildUrdfRobot(urdfString, { log = console.log } = {}) {
    if (!urdfString || typeof THREE === 'undefined') return null;

    const parser = new DOMParser();
    const urdfDoc = parser.parseFromString(urdfString, 'text/xml');
    const robotEl = urdfDoc.querySelector('robot');

    if (!robotEl) {
        log('No robot element found in URDF');
        return null;
    }

    const robotName = robotEl.getAttribute('name') || 'robot';
    log(`Loading URDF for robot: ${robotName}`);

    const robot = new THREE.Group();
    robot.name = robotName;
    const links = {};
    const joints = {};
    let disposed = false;

    // Parse material definitions from URDF root
    const materialDefs = {};
    urdfDoc.querySelectorAll('robot > material').forEach(matEl => {
        const matName = matEl.getAttribute('name');
        const colorEl = matEl.querySelector('color');
        if (matName && colorEl && colorEl.getAttribute('rgba')) {
            const rgba = parseVec(colorEl.getAttribute('rgba'));
            materialDefs[matName] = new THREE.Color(rgba[0], rgba[1], rgba[2]);
        }
    });

    function getMaterialColor(materialEl) {
        if (!materialEl) return DEFAULT_COLOR;

        const colorEl = materialEl.querySelector('color');
        if (colorEl && colorEl.getAttribute('rgba')) {
            const rgba = parseVec(colorEl.getAttribute('rgba'));
            return new THREE.Color(rgba[0], rgba[1], rgba[2]);
        }

        const matName = materialEl.getAttribute('name');
        if (matName && materialDefs[matName]) {
            return materialDefs[matName];
        }

        return DEFAULT_COLOR;
    }

    function makeMaterial(color) {
        return new THREE.MeshStandardMaterial({
            color: color,
            metalness: 0.3,
            roughness: 0.7
        });
    }

    // Create geometry from URDF visual element
    function createGeometry(geometryEl, origin, material) {
        const box = geometryEl.querySelector('box');
        const cylinder = geometryEl.querySelector('cylinder');
        const sphere = geometryEl.querySelector('sphere');
        const meshEl = geometryEl.querySelector('mesh');

        let geom = null;

        if (box) {
            const size = parseVec(box.getAttribute('size'));
            geom = new THREE.BoxGeometry(size[0], size[1], size[2]);
        } else if (cylinder) {
            const radius = parseFloat(cylinder.getAttribute('radius'));
            const length = parseFloat(cylinder.getAttribute('length'));
            geom = new THREE.CylinderGeometry(radius, radius, length, 16);
        } else if (sphere) {
            const radius = parseFloat(sphere.getAttribute('radius'));
            geom = new THREE.SphereGeometry(radius, 16, 16);
        } else if (meshEl) {
            // Load OBJ mesh asynchronously
            const filename = meshEl.getAttribute('filename');
            const scaleAttr = meshEl.getAttribute('scale');

            if (filename && filename.startsWith(MESH_PACKAGE_PREFIX) && filename.endsWith('.obj')) {
                const meshUrl = `/meshes/${filename.replace(MESH_PACKAGE_PREFIX, '')}`;

                // Placeholder group the mesh is added to when loaded
                const placeholder = new THREE.Group();

                if (typeof THREE.OBJLoader !== 'undefined') {
                    const loader = new THREE.OBJLoader();
                    loader.load(
                        meshUrl,
                        (obj) => {
                            const matColor = getMaterialColor(material);
                            obj.traverse((child) => {
                                if (child.isMesh) {
                                    if (child.material) {
                                        const old = Array.isArray(child.material) ? child.material : [child.material];
                                        old.forEach(m => m.dispose());
                                    }
                                    child.material = makeMaterial(matColor);
                                }
                            });

                            // Robot was disposed while the mesh was loading
                            if (disposed) {
                                disposeObject3D(obj);
                                return;
                            }

                            if (scaleAttr) {
                                const scaleVals = parseVec(scaleAttr);
                                obj.scale.set(scaleVals[0], scaleVals[1], scaleVals[2]);
                            }

                            applyOrigin(obj, origin);
                            placeholder.add(obj);
                        },
                        undefined,
                        (error) => console.warn('Failed to load mesh:', meshUrl, error)
                    );
                }

                return placeholder;
            } else {
                // Fallback placeholder for unsupported mesh formats
                geom = new THREE.BoxGeometry(0.03, 0.03, 0.03);
            }
        }

        if (!geom) return null;

        const mesh = new THREE.Mesh(geom, makeMaterial(getMaterialColor(material)));
        applyOrigin(mesh, origin);
        return mesh;
    }

    // Parse links
    const childLinks = new Set();
    urdfDoc.querySelectorAll('link').forEach(linkEl => {
        const linkName = linkEl.getAttribute('name');
        const linkGroup = new THREE.Group();
        linkGroup.name = linkName;

        linkEl.querySelectorAll('visual').forEach(visual => {
            const geometry = visual.querySelector('geometry');
            if (!geometry) return;
            const mesh = createGeometry(geometry, visual.querySelector('origin'), visual.querySelector('material'));
            if (mesh) {
                linkGroup.add(mesh);
            }
        });

        links[linkName] = linkGroup;
    });

    // Parse joints to build hierarchy
    urdfDoc.querySelectorAll('joint').forEach(jointEl => {
        const jointName = jointEl.getAttribute('name');
        const parent = jointEl.querySelector('parent')?.getAttribute('link');
        const child = jointEl.querySelector('child')?.getAttribute('link');
        const axis = jointEl.querySelector('axis');

        joints[jointName] = {
            type: jointEl.getAttribute('type'),
            parent: parent,
            child: child,
            axis: axis && axis.getAttribute('xyz') ? parseVec(axis.getAttribute('xyz')) : [0, 0, 1]
        };

        if (parent && child && links[parent] && links[child]) {
            childLinks.add(child);
            applyOrigin(links[child], jointEl.querySelector('origin'));
            links[parent].add(links[child]);
        }
    });

    // Add root links to robot
    Object.keys(links).forEach(linkName => {
        if (!childLinks.has(linkName)) {
            robot.add(links[linkName]);
        }
    });

    // Rotate for ROS -> Three.js coordinate conversion (Z-up to Y-up)
    robot.rotation.x = -Math.PI / 2;

    log(`URDF loaded: ${Object.keys(links).length} links, ${Object.keys(joints).length} joints`);

    return {
        robot,
        name: robotName,
        links,
        joints,
        dispose() {
            disposed = true;
            if (robot.parent) robot.parent.remove(robot);
            disposeObject3D(robot);
        },
    };
}

/**
 * Apply a TF transform to a link (exact frame name, then without leading slash)
 * @returns {boolean} true if a link was updated
 */
export function applyLinkTransform(links, childFrameId, transform) {
    if (!links || !transform) return false;

    let link = links[childFrameId];
    if (!link && childFrameId) {
        link = links[childFrameId.replace(/^\//, '')];
    }
    if (!link) return false;

    // Apply position
    if (transform.position) {
        link.position.set(transform.position.x, transform.position.y, transform.position.z);
    }

    // Apply rotation (quaternion)
    if (transform.quaternion) {
        link.quaternion.set(
            transform.quaternion.x,
            transform.quaternion.y,
            transform.quaternion.z,
            transform.quaternion.w
        );
    }
    return true;
}
