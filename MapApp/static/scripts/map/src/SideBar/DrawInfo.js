class DrawInfo
{
    constructor() {
        this.htmlId = 'sideBar-right-drawInfo-content';
        this.layerList = new SmartList();
        // M.addMapCallback('pm:create', this.createLayer.bind(this));

        M.DRAW_LAYERS.addInfoAddCallback(this.addLayerCallback.bind(this));
        M.DRAW_LAYERS.addInfoChangeCallback(this.updateLayerCallback.bind(this));
    }    

    addLayerCallback(myargs, args) {

        if (args[1] == 'add') {
            let info = M.DRAW_LAYERS.getDictById(args[0]);
            info.drawManager.instance.drawInfoAdd(this.htmlId, info);
        } else if (args[1] == 'remove') {
            console.log("DrawInfo: removeCallback");
            console.log(args[0]);
            let info = M.DRAW_LAYERS.getDictById(args[0]);
            console.log(info)
            if (info != null) {
                info.drawManager.instance.drawInfoRemove(this.htmlId + '-' + info.id);
            }
        }
    }

    updateLayerCallback(myargs, args) {
        let info = M.DRAW_LAYERS.getDictById(args[0]);
        if (info != null) {
            info.drawManager.instance.drawInfoAdd(this.htmlId, info);
        }
    }
}