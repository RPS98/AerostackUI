
// List class of [type, fileName]
let blocksClassList = [
    ['button', Button],
    ['label', Label],
    ['collapse', Collapse],
]

class HTMLUtils
{

    static _addAttribute(element, id, attributes) {
        if (id != 'none') {
            element.setAttribute('id', id);
        }
        for (let key in attributes) {
            element.setAttribute(key, attributes[key]);
        }
        return element;
    }    

    static addDict(type, id='none', attributes={}, ...args) {
        for (let i=0; i<blocksClassList.length; i++) {
            if (blocksClassList[i][0] == type) {
                return blocksClassList[i][1].addDict(type, id, attributes, ...args);
            }
        }
    }

    static addHTML(parent, childDict) {
        let child = null;
        let flag = true;
        for (let i=0; i<blocksClassList.length; i++) {
            if (blocksClassList[i][0] == childDict.type) {
                child = blocksClassList[i][1].addHTML(childDict.content);
                HTMLUtils._addAttribute(child, childDict.id, childDict.attributes);
                parent.appendChild(child);
                flag = false;
                break; // If types are unique, break after first match to optimize performance
            }
        }
        if (flag) {
            throw new Error('Unknown type of HTML block');
        }
    }

    static addToExistingElement(id, blockList)
    {
        let parent = document.getElementById(id);

        for (let i=0; i<blockList.length; i++) {
            HTMLUtils.addHTML(parent, blockList[i]);
        }
    }
}

