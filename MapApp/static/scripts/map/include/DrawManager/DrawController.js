class DrawController
{
    /**
     * Set default mode
     */
    static drawMouse(){
        M.MAP.pm.disableDraw();
        M.MAP.pm.disableGlobalEditMode();
        M.MAP.pm.disableGlobalRemovalMode();
        M.MAP.pm.disableGlobalDragMode();
        M.MAP.pm.disableGlobalRotateMode();
    }

    /**
     * Set edit mode
     */
    static drawEdit(){
        M.MAP.pm.toggleGlobalEditMode();
    }
    
    /**
     * Set remove mode
     */
    static drawDelete(){
        M.MAP.pm.toggleGlobalRemovalMode();
    }
    
    /**
     * Set drag mode
     */
    static drawMove(){
        M.MAP.pm.toggleGlobalDragMode();
    }
    
    /**
     * Set rotate mode
     */
    static drawRotate(){
        M.MAP.pm.toggleGlobalRotateMode();
    }
    
    /**
     * Remove all layers
     */
    static drawRemoveAll() {

        console.log('Remove all layers');

        // get all layers
        let pm_layers = L.PM.Utils.findLayers(M.MAP);

        // for each layers
        for (let i = 0; i < pm_layers.length; i++) {
            // TODO
            // // avoid detele the image overlay
            // if(pm_layers[i].pm.options['author'] == 'webpage'){
            //     // get the layer by index and remove it
            //     pm_layers[i].remove();
            // }
        }
    }
}