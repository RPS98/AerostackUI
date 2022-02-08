class Li extends HTMLUtilsPrototype 
{
    static addDict(type, id='none', attributes={}, ...args){
        return super.setDict(
            type,
            id,
            attributes,
            {
                'element': args[0],
            }
        );
    } 

    static addHTML(content) {
        let label = document.createElement('LI');
        HTMLUtils.addHTML(label, content.element);
        return label;
    }
}