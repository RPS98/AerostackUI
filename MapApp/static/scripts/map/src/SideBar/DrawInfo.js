class DrawInfo
{
    constructor() {
        this.htmlId = 'sideBar-right-drawInfo-content';
        this.layerList = new SmartList();
        // M.addMapCallback('pm:create', this.createLayer.bind(this));

        M.DRAW_LAYERS.addInfoAddCallback(this.updateLayerCallback.bind(this));
    }

    updateHeightRangeCallback(myargs, args) {
        let info = myargs;
        info.drawManager.height = [args.heightMin, args.heightMax];
    }

    updateLayerCallback(myargs, args) {
        console.log("DrawInfo: updateLayerCallback")
        console.log(args)

        if (args[1] == 'add') {
            // Get layer
            let info = M.DRAW_LAYERS.getDictById(args[0]);

            // Create html associated to layer
            let drawInfo = info.drawManager.instance.getHtmlDrawInfo(this.htmlId, info.layer);
            HTMLUtils.addToExistingElement(`${this.htmlId}`, [drawInfo]);

            // Initialize uav picker
            M.uavPickerInitiliazeCallback(`${this.htmlId}-UAVPicker`);
            let input = document.getElementById(`${this.htmlId}-UAVPicker-auto-Input`);
            if (input != null) {
                input.setAttribute('checked', true);
            }

            // Add callback to height range button
            Utils.addFormCallback(`${this.htmlId}-heighRangeBtn`, [`${this.htmlId}-heightInputMin`, `${this.htmlId}-heightInputMax`], ['heightMin', 'heightMax'], this.updateHeightRangeCallback.bind(this), info);
        }
    }

    createLayer(myargs, layer) {

        let options = layer.layer.pm.options.DrawManager;

        if (options.missionPlanner) {
            let drawManager = options.instance;
            let drawInfo = drawManager.getHtmlDrawInfo(this.htmlId, layer);
            HTMLUtils.addToExistingElement(`${this.htmlId}`, [drawInfo]);
        }
    }
}