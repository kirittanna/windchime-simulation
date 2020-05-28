import * as THREE from "three/build/three.module";
import { AmmoDebugDrawer, AmmoDebugConstants, DefaultBufferSize } from "ammo-debug-drawer";
import { OrbitControls } from './OrbitControls';

Ammo().then(function(Ammo) {

    // Detects webgl
    if ( ! Detector.webgl ) {
        Detector.addGetWebGLMessage();
        document.getElementById( 'main' ).innerHTML = "";
    }

    // - Global variables -
    var softBodyHelpers;
    var sail;

    // Graphics variables
    var container, stats;
    var camera, controls, scene, renderer;
    var textureLoader;
    var clock = new THREE.Clock();

    // Physics variables
    var gravityConstant = -9.8;
    var collisionConfiguration;
    var dispatcher;
    var broadphase;
    var solver;
    var softBodySolver;
    var physicsWorld;
    var rigidBodies = [];
    var margin = 0.05;
    var hinge;
    var group;
    var ropes = [];
    var transformAux1 = new Ammo.btTransform();
    var debugDrawer;
    var debugGeometry;

    var time = 0;
    var armMovement = 0;

    // - Main code -

    init();
    animate();

    // - Functions -

    function init() {
        initGraphics();
        initPhysics();
        initDebug();
        createObjects();
        initInput();
    }

    function initGraphics() {

        container = document.getElementById( 'main' );

        camera = new THREE.PerspectiveCamera( 60, window.innerWidth / window.innerHeight, 0.2, 2000 );

        scene = new THREE.Scene();

        camera.position.x = -7;
        camera.position.y = 10;
        camera.position.z =  -20;

        renderer = new THREE.WebGLRenderer();
        renderer.setClearColor( 0xbfd1e5 );
        renderer.setPixelRatio( window.devicePixelRatio );
        renderer.setSize( window.innerWidth, window.innerHeight );
        renderer.shadowMap.enabled = true;

        controls = new OrbitControls( camera, renderer.domElement );
        controls.target.y = 2;

        textureLoader = new THREE.TextureLoader();

        var ambientLight = new THREE.AmbientLight( 0x404040 );
        scene.add( ambientLight );

        var light = new THREE.DirectionalLight( 0xffffff, 1 );
        light.position.set( -10, 10, 5 );
        light.castShadow = true;
        var d = 10;
        light.shadow.camera.left = -d;
        light.shadow.camera.right = d;
        light.shadow.camera.top = d;
        light.shadow.camera.bottom = -d;

        light.shadow.camera.near = 2;
        light.shadow.camera.far = 50;

        light.shadow.mapSize.x = 1024;
        light.shadow.mapSize.y = 1024;

        scene.add( light );

        container.innerHTML = "";
        container.appendChild( renderer.domElement );

        stats = new Stats();
        stats.domElement.style.position = 'absolute';
        stats.domElement.style.top = '0px';
        container.appendChild( stats.domElement );

        //

        window.addEventListener( 'resize', onWindowResize, false );

    }

    function initPhysics() {

        // Physics configuration

        collisionConfiguration = new Ammo.btSoftBodyRigidBodyCollisionConfiguration();
        dispatcher = new Ammo.btCollisionDispatcher( collisionConfiguration );
        broadphase = new Ammo.btDbvtBroadphase();
        solver = new Ammo.btSequentialImpulseConstraintSolver();
        softBodySolver = new Ammo.btDefaultSoftBodySolver();
        physicsWorld = new Ammo.btSoftRigidDynamicsWorld( dispatcher, broadphase, solver, collisionConfiguration, softBodySolver);
        physicsWorld.setGravity( new Ammo.btVector3( 0, gravityConstant, 0 ) );
        physicsWorld.getWorldInfo().set_m_gravity( new Ammo.btVector3( 0, gravityConstant, 0 ) );
    }

    function initDebug() {
        var debugVertices = new Float32Array(DefaultBufferSize);
        var debugColors = new Float32Array(DefaultBufferSize);
        debugGeometry = new THREE.BufferGeometry();
        var vertAttr = new THREE.BufferAttribute(debugVertices, 3);
        // vertAttr.usage = THREE.DynamicReadUsage;
        debugGeometry.setAttribute("position", vertAttr);
        var colorAttr = new THREE.BufferAttribute(debugColors, 3);
        // colorAttr.usage = THREE.DynamicReadUsage;
        debugGeometry.setAttribute("color", colorAttr);
        var debugMaterial = new THREE.LineBasicMaterial({ vertexColors: THREE.VertexColors });
        var debugMesh = new THREE.LineSegments(debugGeometry, debugMaterial);
        debugMesh.frustumCulled = false;
        scene.add(debugMesh);
        debugDrawer = new AmmoDebugDrawer(null, debugVertices, debugColors, physicsWorld);
        debugDrawer.enable();
        debugDrawer.setDebugMode(AmmoDebugConstants.NoDebug);
    }

    function createObjects() {

        var pos = new THREE.Vector3();
        var quat = new THREE.Quaternion();

        group = new THREE.Group();
        group.position.set(0, 0, 0);
        scene.add( group );

        // Ground
        pos.set( 0, - 0.05, 0 );
        quat.set( 0, 0, 0, 1 );
        var ground = createParalellepiped( 40, 0.1, 40, 0, pos, quat, new THREE.MeshPhongMaterial( { color: 0xFFFFFF } ) );
        ground.castShadow = true;
        ground.receiveShadow = true;

        // Windchime
        // Hook
        pos.set( 0, 8.85, 0 );
        quat.set( 0, 0, 0, 1 );
        var hook = createParalellepiped(0.25, 0.25, 0.25, 0, pos, quat, new THREE.MeshLambertMaterial( {color: 0xff00ff} ));

        // Umbrella
        var coneMass = 60.0;
        var coneRadius = 3;
        var coneHeight = 1;
        var cone = new THREE.Mesh(
            new THREE.ConeGeometry( coneRadius, coneHeight, 8 ), 
            new THREE.MeshLambertMaterial( {color: 0xffff00, side: THREE.DoubleSide} )
        );
        var coneShape = new Ammo.btConeShape(coneRadius, coneHeight);
        pos.set( 0, 10.1, 0 );
        quat.set( 0, 0, 0, 1 );
        createRigidBody( cone, coneShape, coneMass, pos, quat );

        var pivotA = new Ammo.btVector3( 0, 10.6, 0 );
        var pivotB = new Ammo.btVector3( 0, 9.975, 0 );
        var constraint = new Ammo.btPoint2PointConstraint( cone.userData.physicsBody, hook.userData.physicsBody, pivotA, pivotB );
        physicsWorld.addConstraint( constraint, true );

        // Hangings
        var count = 8;
        var TWO_PI = Math.PI * 2;
        var angle = 0;
        var angle_incr = TWO_PI / count;
        var tubeMass = 3.0;
        var hangingsRadius = 1.5;
        for(var i = 0; i < count; i++) {
            var tube_geometry = new THREE.CylinderBufferGeometry( 0.1, 0.1, 4, 8 );
            var tube_material = new THREE.MeshLambertMaterial( {color: 0x00ffff} );
            var tube = new THREE.Mesh( tube_geometry, tube_material );
            var tubeShape = new Ammo.btCylinderShape(
                new Ammo.btVector3(0.1, 2, 0.1)
            );
            var compound = new Ammo.btCompoundShape();
            var localTrans = new Ammo.btTransform();
            localTrans.setIdentity();
            localTrans.setOrigin(new Ammo.btVector3(0,0,0));
            compound.addChildShape(localTrans, tubeShape);

            angle += angle_incr;
            var pos_x = Math.cos(angle) * hangingsRadius;
            var pos_z = Math.sin(angle) * hangingsRadius;
            pos.set( pos_x, 0, pos_z );
            quat.set( 0, 0, 0, 1 );
            createRigidBody(tube, compound, tubeMass, pos, quat);

            var pivotA = new Ammo.btVector3( 0, 2, 0 );
            var pivotB = new Ammo.btVector3( pos_x, -0.5, pos_z );
            var pointConstraint = new Ammo.btPoint2PointConstraint( tube.userData.physicsBody, cone.userData.physicsBody, pivotA, pivotB );
            physicsWorld.addConstraint( pointConstraint, true );
        }

        // Clapper
        var clapperMass = 10;
        var clapperHeight = 0.1;
        var clapper_geometry = new THREE.CylinderBufferGeometry( 1, 1, 0.1, 16 );
        var clapper_material = new THREE.MeshLambertMaterial( {color: 0xff0000} );
        var clapper = new THREE.Mesh( clapper_geometry, clapper_material );
        var clapperShape = new Ammo.btCylinderShape(
            new Ammo.btVector3(1, 0.05, 1)
        );
        pos.set( 0, 7, 0 );
        quat.set( 0, 0, 0, 1 );
        createRigidBody(clapper, clapperShape, clapperMass, pos, quat);

        // Sail
        var sailMass = 1.0;
        var sailHeight = 0.5;
        sail = new THREE.Mesh( new THREE.PlaneGeometry( sailHeight, sailHeight ), new THREE.MeshPhongMaterial( { color: 0x202020, side: THREE.DoubleSide } ) );
        sail.castShadow = true;
        sail.receiveShadow = true;
        var sailShape = new Ammo.btBoxShape( new Ammo.btVector3(sailHeight * 0.5, sailHeight * 0.5, 0.01) );
        sailShape.setMargin( margin );
        pos.set( 0, 3.5, 0 );
        quat.set( 0, 0, 0, 1 );
        createRigidBody( sail, sailShape, sailMass, pos, quat );
        sail.userData.physicsBody.setFriction( 0.5 );

        // The ropes
        softBodyHelpers = new Ammo.btSoftBodyHelpers();
        var rope1Pos = new THREE.Vector3(0, 3.25, 0);
        var rope1 = createRope(6, 4 - sailHeight * 0.5 - clapperHeight * 0.5, 4, rope1Pos);
        var rope2Pos = new THREE.Vector3(0, 7 + 0.025, 0);
        var rope2 = createRope(6, 4.1 - clapperHeight * 0.5 - coneHeight * 0.5, 8, rope2Pos);

        ropes.push(rope1, rope2);

        // Glue the rope extremes to the sail and the arm
        var influence = 1;
        rope1.userData.physicsBody.appendAnchor( 0, sail.userData.physicsBody, new Ammo.btVector3(0, sailHeight * 0.5, 0), true, influence );
        rope1.userData.physicsBody.appendAnchor( 6, clapper.userData.physicsBody, true, influence );
        rope2.userData.physicsBody.appendAnchor( 0, clapper.userData.physicsBody, true, influence );
        rope2.userData.physicsBody.appendAnchor( 6, cone.userData.physicsBody, true, influence );
    }

    function createRope(ropeNumSegments, ropeLength, ropeMass, ropePos) {
        var segmentLength = ropeLength / ropeNumSegments;
        var ropeGeometry = new THREE.BufferGeometry();
        var ropeMaterial = new THREE.LineBasicMaterial( { color: 0x000000 } );
        var ropePositions = [];
        var ropeIndices = [];

        for ( var i = 0; i < ropeNumSegments + 1; i++ ) {
            ropePositions.push( ropePos.x, ropePos.y + i * segmentLength, ropePos.z );
        }

        for ( var i = 0; i < ropeNumSegments; i++ ) {
            ropeIndices.push( i, i + 1 );
        }

        ropeGeometry.setIndex( new THREE.BufferAttribute( new Uint16Array( ropeIndices ), 1 ) );
        ropeGeometry.setAttribute( 'position', new THREE.BufferAttribute( new Float32Array( ropePositions ), 3 ) );
        ropeGeometry.computeBoundingSphere();
        var threeObject = new THREE.LineSegments( ropeGeometry, ropeMaterial );
        // threeObject.castShadow = true;
        // threeObject.receiveShadow = true;
        group.add( threeObject );

        // Rope physic object
        var ropeStart = new Ammo.btVector3( ropePos.x, ropePos.y, ropePos.z );
        var ropeEnd = new Ammo.btVector3( ropePos.x, ropePos.y + ropeLength, ropePos.z );
        var ropeSoftBody = softBodyHelpers.CreateRope( physicsWorld.getWorldInfo(), ropeStart, ropeEnd, ropeNumSegments - 1, 0 );
        var sbConfig = ropeSoftBody.get_m_cfg();
        sbConfig.set_viterations( 10 );
        sbConfig.set_piterations( 10 );
        ropeSoftBody.setTotalMass( ropeMass, false );
        Ammo.castObject( ropeSoftBody, Ammo.btCollisionObject ).getCollisionShape().setMargin( margin * 3 );
        physicsWorld.addSoftBody( ropeSoftBody, 1, -1 );
        threeObject.userData.physicsBody = ropeSoftBody;
        // Disable deactivation
        ropeSoftBody.setActivationState( 4 );

        return threeObject;
    }

    function createParalellepiped( sx, sy, sz, mass, pos, quat, material ) {

        var threeObject = new THREE.Mesh( new THREE.BoxGeometry( sx, sy, sz, 1, 1, 1 ), material );
        var shape = new Ammo.btBoxShape( new Ammo.btVector3( sx * 0.5, sy * 0.5, sz * 0.5 ) );
        shape.setMargin( margin );

        createRigidBody( threeObject, shape, mass, pos, quat );

        return threeObject;

    }

    function createRigidBody( threeObject, physicsShape, mass, pos, quat ) {

        threeObject.position.copy( pos );
        threeObject.quaternion.copy( quat );

        var transform = new Ammo.btTransform();
        transform.setIdentity();
        transform.setOrigin( new Ammo.btVector3( pos.x, pos.y, pos.z ) );
        transform.setRotation( new Ammo.btQuaternion( quat.x, quat.y, quat.z, quat.w ) );
        var motionState = new Ammo.btDefaultMotionState( transform );

        var localInertia = new Ammo.btVector3( 0, 0, 0 );
        physicsShape.calculateLocalInertia( mass, localInertia );

        var rbInfo = new Ammo.btRigidBodyConstructionInfo( mass, motionState, physicsShape, localInertia );
        var body = new Ammo.btRigidBody( rbInfo );

        threeObject.userData.physicsBody = body;

        group.add( threeObject );

        if ( mass > 0 ) {
            rigidBodies.push( threeObject );

            // Disable deactivation
            body.setActivationState( 4 );
        }

        physicsWorld.addRigidBody( body );

    }

    function createRandomColor() {
        return Math.floor( Math.random() * ( 1 << 24 ) );
    }

    function createMaterial() {
        return new THREE.MeshPhongMaterial( { color: createRandomColor() } );
    }

    function initInput() {
        window.addEventListener('keyup', function(e) {
            if(e.keyCode === 32) {
                sail.userData.physicsBody.applyCentralImpulse(
                    new Ammo.btVector3(
                        Math.random() * 8 - 4,
                        Math.random() * 8 - 4,
                        Math.random() * 8 - 4
                    )
                );
            }
        });
    }

    function onWindowResize() {
        camera.aspect = window.innerWidth / window.innerHeight;
        camera.updateProjectionMatrix();
        renderer.setSize( window.innerWidth, window.innerHeight );
    }

    function animate() {
        requestAnimationFrame( animate );
        render();
        stats.update();
    }

    function render() {

        var deltaTime = clock.getDelta();

        updatePhysics( deltaTime );

        controls.update( deltaTime );

        renderer.render( scene, camera );

        time += deltaTime;

    }

    function updatePhysics( deltaTime ) {

        // Step world
        physicsWorld.stepSimulation( deltaTime, 10 );

        if (debugDrawer) {
            if (debugDrawer.index !== 0) {
                debugGeometry.attributes.position.needsUpdate = true;
                debugGeometry.attributes.color.needsUpdate = true;
            }
            debugGeometry.setDrawRange(0, debugDrawer.index);
            debugDrawer.update();
        }

        // Update rope
        for (let r = 0; r < ropes.length; r++) {
            const rope = ropes[r];
            var softBody = rope.userData.physicsBody;
            var ropePositions = rope.geometry.attributes.position.array;
            var numVerts = ropePositions.length / 3;
            var nodes = softBody.get_m_nodes();
            var indexFloat = 0;
            for ( var i = 0; i < numVerts; i ++ ) {

                var node = nodes.at( i );
                var nodePos = node.get_m_x();
                ropePositions[ indexFloat++ ] = nodePos.x();
                ropePositions[ indexFloat++ ] = nodePos.y();
                ropePositions[ indexFloat++ ] = nodePos.z();

            }
            rope.geometry.attributes.position.needsUpdate = true;
        }

        // Update rigid bodies
        for ( var i = 0, il = rigidBodies.length; i < il; i++ ) {
            var objThree = rigidBodies[ i ];
            var objPhys = objThree.userData.physicsBody;
            var ms = objPhys.getMotionState();
            if ( ms ) {

                ms.getWorldTransform( transformAux1 );
                var p = transformAux1.getOrigin();
                var q = transformAux1.getRotation();
                objThree.position.set( p.x(), p.y(), p.z() );
                objThree.quaternion.set( q.x(), q.y(), q.z(), q.w() );

            }
        }

    }
});