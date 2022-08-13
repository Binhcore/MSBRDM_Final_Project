function [Y,parameter] = regressor(M,C,G,varargin)
%Code mainly from Simon's tutorial session screenshot
%Update usage of children function:
%https://de.mathworks.com/help/symbolic/sym.children.html
%Starting in R2020b, the syntax subexpr = children(expr) for a scalar input expr returns subexpr as a nonnested cell array instead of a vector. You can use subexpr = children(expr,ind) to index into the returned cell array of subexpressions. For more information, see Compatibility Considerations.
%To convert the cell array of subexpressions into a vector, you can use the command [subexpr{:}].
%V = [subexpr{:}]
    p = inputParser;
    addRequired(p, 'M');
    addRequired(p, 'C');
    addRequired(p, 'G');
    addOptional(p, 'errorspace', false);
    parse(p, M, C, G, varargin{:});
    errorspace = p.Results.errorspace;

    parameter=[];

    n=size(M,1);

    q=sym('q%d',[n,1],'real');
    qp=sym('qp%d',[n,1],'real');
    qpp=sym('qpp%d',[n,1],'real');

    state_var=[q', qp',qpp'];

    if(errorspace ==true)
        qrp = sym('qrp%d', [n,1], 'real');
        qrpp = sym('qrpp%d', [n,1], 'real');
        state_var =[state_var,qrp',qrpp'];
    end

    if(errorspace==true)
        equ=expand(M*qrpp + C*qrp+G);
    else
        equ=expand(M*qpp + C*qp+G);
    end

    for i = 1: size(equ,1)
        expr = children(equ(i));
        expr = [expr{:}];

        for j = i:size(expr,2)
            cut_expr = remove_term(expr(j),state_var);
            parameter = [parameter, cut_expr];
        end
    end

    parameter = unique(parameter);
    r = size(parameter,2);

    z = sym('z%d',[1,r], 'real');

    equ = subs(equ, parameter, z);

    Y=simplify(equationsToMatrix(equ,z));
    parameter = parameter.';
end

function [exp, rm_exp] = remove_term(exp,varlist)
    subexp = children(exp);
    subexp = [subexp{:}];
    rm_exp=[];
    rm_idx=[];

    for i = 1:size(subexp,2)
        var = symvar(subexp(i));
        flag=false;

        if size(var)>0
            for j=1:size(varlist,2)
                if var ==varlist(j)
                    flag = true;
                    break;
                end
            end
        else
            flag = true;
        end

        if flag == true
            rm_exp=[rm_exp, subexp(i)];
            rm_idx=[rm_idx,i];
        end
    end

    subexp(rm_idx)=[];
    exp=prod(subexp);
end

