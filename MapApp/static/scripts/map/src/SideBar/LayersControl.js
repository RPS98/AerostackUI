class LayersControl
{
    constructor() {
        M.DRAW_LAYERS.addInfoAddCallback(this.addDrawLayerCallback.bind(this));
        M.MISSION_LAYERS.addInfoAddCallback(this.addMissionLayerCallback.bind(this));
        M.UAV_LAYERS.addInfoAddCallback(this.addUavLayerCallback.bind(this));

        this.drawControlLayer = L.layerGroup();
        this.drawControlLayer.addTo(M.MAP);
        M.layerControl.addOverlay(this.drawControlLayer, 'Draw Layers');
        this.uavControlLayers = new SmartList();
        this.missionControlLayers = new SmartList();

    }

    addDrawLayerCallback(myargs, args) {
        let layerId = args[0];
        
        if (args[1] == 'add') {
            let info = M.DRAW_LAYERS.getDictById(layerId);
            this.drawControlLayer.addLayer(info.layer);
        }
    }

    addMissionLayerCallback(myargs, args) {
        let layerId = args[0];
        
        if (args[1] == 'add') {
            let info = M.MISSION_LAYERS.getDictById(layerId);
            let missionId = info.drawManager.options.missionId;
            let dict = this.missionControlLayers.getDictById(missionId);
            if (dict == null) {
                let missionControlLayer = L.layerGroup();
                missionControlLayer.addTo(M.MAP);
                M.layerControl.addOverlay(missionControlLayer, missionId);

                this.missionControlLayers.addObject(missionId, {'controlLayer': missionControlLayer});
            }
            dict = this.missionControlLayers.getDictById(missionId);
            dict.controlLayer.addLayer(info.layer);
        }
    }

    addUavLayerCallback(myargs, args) {
        let layerId = args[0];
        
        if (args[1] == 'add') {
            let info = M.UAV_LAYERS.getDictById(layerId);
            let uavId = info.drawManager.options.uavList[0];
            let dict = this.uavControlLayers.getDictById(uavId);
            if (dict == null) {
                let uavControlLayer = L.layerGroup();
                uavControlLayer.addTo(M.MAP);
                M.layerControl.addOverlay(uavControlLayer, uavId);

                this.uavControlLayers.addObject(uavId, {'controlLayer': uavControlLayer});
            }
            dict = this.uavControlLayers.getDictById(uavId);
            dict.controlLayer.addLayer(info.layer);

        }
    }
} 