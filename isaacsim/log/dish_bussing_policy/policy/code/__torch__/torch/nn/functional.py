def elu(input: Tensor,
    alpha: float=1.,
    inplace: bool=False) -> Tensor:
  if inplace:
    result = torch.elu_(input, alpha)
  else:
    result = torch.elu(input, alpha)
  return result
