/**
 * Draw Controller for the user to draw on the map.
 */
class DrawController {
    /**
     * Disable all draw modes
     * @return {void}
     * @access public
     * @static
     */
    static drawMouse() {
        M.MAP.pm.disableDraw();
        M.MAP.pm.disableGlobalEditMode();
        M.MAP.pm.disableGlobalRemovalMode();
        M.MAP.pm.disableGlobalDragMode();
        M.MAP.pm.disableGlobalRotateMode();
    }

    /**
     * Enable edit mode
     * @return {void}
     * @access public
     * @static
     */
    static drawEdit() {
        M.MAP.pm.toggleGlobalEditMode();
    }

    /**
     * Enable delete mode
     * @return {void}
     * @access public
     * @static
     */
    static drawDelete() {
        M.MAP.pm.toggleGlobalRemovalMode();
    }

    /**
     * Enable remove mode
     * @return {void}
     * @access public
     * @static
     */
    static drawMove() {
        M.MAP.pm.toggleGlobalDragMode();
    }

    /**
     * Enable rotate mode
     * @return {void}
     * @access public
     * @static
     */
    static drawRotate() {
        M.MAP.pm.toggleGlobalRotateMode();
    }

    /**
     * Remove all draw layers that are not confirmed in a mission
     * @return {void}
     * @access public
     * @static
     */
    static drawRemoveAll() {
        let layers = M.getLayers();

        for (let i = 0; i < layers.length; i++) {
            if (layers[i].pm.options.status == 'draw') {
                layers[i].remove();
            }
        }
    }
}