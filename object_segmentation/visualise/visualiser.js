function fillCanvases(){
    var r1 = new renderContainer('clouds/color.pcd','canvas_cloud');
    r1.renderView();
    
}

function renderContainer(pcd_file,canvas_name){
    var ps,cloud;
    this.pcd_file = pcd_file;
    this.canvas_name = canvas_name;

    // Create an orbit camera halfway between the closest and farthest point
    var cam = new OrbitCam({closest:-5, farthest:5, distance: 5});
    var isDragging = false;
    var rotationStartCoords = [0, 0];

    this.vualize = function (){
	this.pcd_file = document.getElementById('PCDPath').value;
	this.renderView(pcd_file);
    }
    
    this.zoom = function (amt){
	cam.goFarther(amt);
    }
    
    this.mousePressed = function (){
	rotationStartCoords[0] = ps.mouseX;
	rotationStartCoords[1] = ps.mouseY;
	
	isDragging = true;
    }
    
    this.mouseReleased = function (){
	isDragging = false;
    }
    
    this.render = function (){
	if(isDragging === true){
	    // how much was the cursor moved compared to last time
	    // this function was called?
	    var deltaX = ps.mouseX - rotationStartCoords[0];
	    var deltaY = ps.mouseY - rotationStartCoords[1];
	    
	    // now that the camera was updated, reset where the
	    // rotation will start for the next time this function is called.
	    rotationStartCoords = [ps.mouseX, ps.mouseY];
	    
	    cam.yaw(-deltaX * 0.015);
	    cam.pitch(deltaY * 0.015);
	}
	
	var c = cloud.getCenter();  
	ps.multMatrix(M4x4.makeLookAt(cam.position, cam.direction, cam.up));
	ps.translate(-cam.position[0]-c[0], -cam.position[1]-c[1], -cam.position[2]-c[2] );
	
	ps.clear();
	ps.render(cloud);
    }
    
    this.renderView = function(){
	console.log(this.pcd_file,this.canvas_name);
	ps = new PointStream();
	
	ps.setup(document.getElementById(canvas_name));
	
	ps.background([0, 0, 0, 1]);
	ps.pointSize(10);
    
	ps.onRender = this.render;
	ps.onMouseScroll = this.zoom;
	ps.onMousePressed = this.mousePressed;
	ps.onMouseReleased = this.mouseReleased;
  
	cloud = ps.load(this.pcd_file);
    }
}