class DirectionalLight {

    constructor(lightIntensity, lightColor, lightPos, focalPoint, lightUp, hasShadowMap, gl) {
        this.mesh = Mesh.cube(setTransform(0, 0, 0, 0.2, 0.2, 0.2, 0));         // light cube size(7 paramter?)
        this.mat = new EmissiveMaterial(lightIntensity, lightColor);
        this.lightPos = lightPos;
        this.focalPoint = focalPoint;
        this.lightUp = lightUp;

        this.hasShadowMap = hasShadowMap;
        this.fbo = new FBO(gl);
        if (!this.fbo) {
            console.log("无法设置帧缓冲区对象");
            return;
        }
    }

    CalcLightMVP(translate, scale) {
        let lightMVP = mat4.create();
        let modelMatrix = mat4.create();
        let viewMatrix = mat4.create();
        let projectionMatrix = mat4.create();

        // Model transform
        mat4.translate(modelMatrix, modelMatrix, translate);
        mat4.scale(modelMatrix, modelMatrix, scale);
        
        // View transform
        mat4.lookAt(viewMatrix, this.lightPos, this.focalPoint, this.lightUp);       // eye position, eye dir, and up dir(y-Axis default when eye dir z-Axis)
        
        // Projection transform
        // * just use orthogonal projection, no need for perspective projection.
        // * create a range box(large enough)
        // * too large => seriously self-hidden
        // * too small => shadow missing  
        // ? why is 500 in zAixs (not -500?)
        mat4.ortho(projectionMatrix, -100, 100, -100, 100, 0, 600);
        // mat4.ortho(projectionMatrix, this.lightPos[0]-120, this.lightPos[0]+120, this.lightPos[1]-120, this.lightPos[1]+120, 0, this.lightPos[2]+500);
        // console.log("lightPos ==> " + String(this.lightPos));
        mat4.multiply(lightMVP, projectionMatrix, viewMatrix);
        mat4.multiply(lightMVP, lightMVP, modelMatrix);

        return lightMVP;
    }
}
