class UavDrawer
{
    constructor() {
        M.UAV_MANAGER.addUavCallback(this.updateUavCallback.bind(this));
        M.UAV_MANAGER.addUavListCallback(this.updateUavListCallback.bind(this));

        this.uavListId = Object.assign([], M.UAV_MANAGER.getUavList());
        this.uavDict = Object.assign({}, M.UAV_MANAGER.getUavDict());

        this.uavLayerDict = {};
    }

    drawUav(id) { 
        let uav = M.UAV_MANAGER.getUavDictById(id);
        
        if (id in this.uavLayerDict) {

            if (uav['state'] != this.uavDict[id]['state']) {
                this.uavLayerDict[id]['marker'].codeLayerDrawn.options['state'] = uav['state'];
            }
            if (uav['pose'] != this.uavDict[id]['pose']) {
                this.uavLayerDict[id]['marker'].codeLayerDrawn.setLatLng([uav['pose']['lat'], uav['pose']['lng']]);
            }
            if (uav['odom'] != this.uavDict[id]['odom']) {
                this.uavLayerDict[id]['odom'].codeLayerDrawn.setLatLngs(uav['odom']);
                // TODO: Update odom
                //this.uavLayerDict[id]['odom'].codeLayerDrawn.setLatLng([uav['odom']['lat'], uav['odom']['lng']]);
            }
            if (uav['desiredPath'] != this.uavDict[id]['desiredPath']) {
                console.log("Changing desiredPath");
                console.log(uav['desiredPath']);
                this.uavLayerDict[id]['desiredPath'].codeLayerDrawn.setLatLngs(uav['desiredPath']);
                // TODO: Update desiredPath
                // this.uavLayerDict[id]['desiredPath'].codeLayerDrawn.setLatLngs(uav['desiredPath']);
            }
            this.uavDict[id] = Object.assign(this.uavDict[id], uav);

        } else {

            let layerPose = new UAVMarker();
            layerPose.codeDraw(id, [uav['pose']['lat'], uav['pose']['lng']], {'state': uav['state']});

            let layerOdom = new Odom();
            console.log("Drawing odom");
            console.log(uav['odom']);
            layerOdom.codeDraw(id, uav['odom']);
            
            let layerDesiredPath = new DesiredPath();
            console.log("Drawing desiredPath");
            console.log(uav['desiredPath']);
            layerDesiredPath.codeDraw(id, uav['desiredPath']);

            this.uavLayerDict[id] = {
                'marker': layerPose,
                'odom': layerOdom,
                'desiredPath': layerDesiredPath,
            };
            this.uavDict[id] = Object.assign({}, uav);
        }
    }

    updateUavCallback(myargs, args) {
        this.drawUav(args);
    }

    updateUavListCallback(myargs, args) {
        for (let i=0; i<args.length; i++) {
            this.drawUav(args[i]);
        }
    }
}