class Form extends HTMLUtilsPrototype 
{
    static addDict(type, id='none', attributes={}, ...args){
        return super.setDict(
            type,
            id,
            attributes,
            {
                'list': args[0],
            }
        );
    }   

    static addHTML(content) {
        let div = document.createElement('div');
        for (let i = 0; i < content.list.length; i++) {
            HTMLUtils.addHTML(div, content.list[i]);
        }
        return div;
    }
}